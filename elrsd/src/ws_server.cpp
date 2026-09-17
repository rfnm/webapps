// elrsd: minimal RFC 6455 WebSocket push server. One thread: accept + client
// poll (close/ping frames answered, everything else from clients is ignored); broadcast()
// writes server->client text frames non-blocking and drops a client whose socket would
// block (slow consumer) - the daemon must never stall on the WS path. Own SHA-1 + base64
// (no OpenSSL on the board). No NEEDS.
#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include "daemon.h"

namespace elrsd {

// ---- SHA-1 (RFC 3174) --------------------------------------------------------------------------
static void sha1(const uint8_t *msg, size_t len, uint8_t out[20]) {
	uint32_t h[5] = { 0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0 };
	std::vector<uint8_t> m(msg, msg + len);
	m.push_back(0x80);
	while(m.size() % 64 != 56) m.push_back(0);
	uint64_t bits = (uint64_t)len * 8;
	for(int i = 7; i >= 0; i--) m.push_back((uint8_t)(bits >> (i * 8)));
	auto rol = [](uint32_t v, int n) { return (v << n) | (v >> (32 - n)); };
	for(size_t off = 0; off < m.size(); off += 64) {
		uint32_t w[80];
		for(int i = 0; i < 16; i++)
			w[i] = ((uint32_t)m[off + i * 4] << 24) | ((uint32_t)m[off + i * 4 + 1] << 16) |
			       ((uint32_t)m[off + i * 4 + 2] << 8) | m[off + i * 4 + 3];
		for(int i = 16; i < 80; i++) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
		uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
		for(int i = 0; i < 80; i++) {
			uint32_t f, k;
			if(i < 20) { f = (b & c) | (~b & d); k = 0x5A827999; }
			else if(i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
			else if(i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
			else { f = b ^ c ^ d; k = 0xCA62C1D6; }
			uint32_t t = rol(a, 5) + f + e + k + w[i];
			e = d; d = c; c = rol(b, 30); b = a; a = t;
		}
		h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
	}
	for(int i = 0; i < 5; i++) {
		out[i * 4] = (uint8_t)(h[i] >> 24); out[i * 4 + 1] = (uint8_t)(h[i] >> 16);
		out[i * 4 + 2] = (uint8_t)(h[i] >> 8); out[i * 4 + 3] = (uint8_t)h[i];
	}
}

static std::string base64(const uint8_t *p, size_t n) {
	static const char *t = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	std::string o;
	for(size_t i = 0; i < n; i += 3) {
		uint32_t v = (uint32_t)p[i] << 16 | (i + 1 < n ? (uint32_t)p[i + 1] << 8 : 0) | (i + 2 < n ? p[i + 2] : 0);
		o += t[(v >> 18) & 63];
		o += t[(v >> 12) & 63];
		o += i + 1 < n ? t[(v >> 6) & 63] : '=';
		o += i + 2 < n ? t[v & 63] : '=';
	}
	return o;
}

std::string ws_accept_key(const std::string &client_key) {   // exposed for the selftest
	std::string s = client_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
	uint8_t d[20];
	sha1((const uint8_t *)s.data(), s.size(), d);
	return base64(d, 20);
}

// ---- server ---------------------------------------------------------------------------------------
struct ws_client {
	int fd;
	bool upgraded = false;
	std::string inbuf;
};

struct ws_server::impl {
	int listen_fd = -1;
	int port = 0;
	std::thread thread;
	volatile bool stop = false;
	std::mutex mtx;                  // guards clients
	std::vector<ws_client> clients;

	static std::string frame(const std::string &payload) {
		std::string f;
		f += (char)0x81;   // FIN + text
		size_t n = payload.size();
		if(n < 126) f += (char)n;
		else if(n < 65536) { f += (char)126; f += (char)(n >> 8); f += (char)(n & 0xFF); }
		else { f += (char)127; for(int i = 7; i >= 0; i--) f += (char)((uint64_t)n >> (i * 8)); }
		return f + payload;
	}

	static bool send_all_nb(int fd, const std::string &data) {
		size_t off = 0;
		while(off < data.size()) {
			ssize_t w = send(fd, data.data() + off, data.size() - off, MSG_DONTWAIT | MSG_NOSIGNAL);
			if(w < 0) return false;   // EAGAIN = slow client -> drop (no partial-frame bookkeeping)
			off += (size_t)w;
		}
		return true;
	}

	void handshake(ws_client &c) {
		size_t end = c.inbuf.find("\r\n\r\n");
		if(end == std::string::npos) {
			if(c.inbuf.size() > 8192) { close(c.fd); c.fd = -1; }
			return;
		}
		std::string hdr = c.inbuf.substr(0, end);
		c.inbuf.erase(0, end + 4);
		std::string key;
		size_t p = 0;
		while(p < hdr.size()) {
			size_t e = hdr.find("\r\n", p);
			if(e == std::string::npos) e = hdr.size();
			std::string line = hdr.substr(p, e - p);
			p = e + 2;
			size_t col = line.find(':');
			if(col == std::string::npos) continue;
			std::string k = line.substr(0, col), v = line.substr(col + 1);
			for(auto &ch : k) ch = (char)tolower((unsigned char)ch);
			while(!v.empty() && (v[0] == ' ' || v[0] == '\t')) v.erase(0, 1);
			while(!v.empty() && (v.back() == ' ' || v.back() == '\r')) v.pop_back();
			if(k == "sec-websocket-key") key = v;
		}
		std::string resp;
		if(key.empty()) {
			resp = "HTTP/1.1 400 Bad Request\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
			send_all_nb(c.fd, resp);
			close(c.fd);
			c.fd = -1;
			return;
		}
		resp = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: " +
		       ws_accept_key(key) + "\r\n\r\n";
		if(!send_all_nb(c.fd, resp)) { close(c.fd); c.fd = -1; return; }
		c.upgraded = true;
	}

	// consume client frames: answer ping, honour close, drop everything else
	void consume(ws_client &c) {
		while(c.inbuf.size() >= 2) {
			uint8_t b0 = (uint8_t)c.inbuf[0], b1 = (uint8_t)c.inbuf[1];
			bool masked = b1 & 0x80;
			uint64_t len = b1 & 0x7F;
			size_t hl = 2;
			if(len == 126) { if(c.inbuf.size() < 4) return; len = ((uint8_t)c.inbuf[2] << 8) | (uint8_t)c.inbuf[3]; hl = 4; }
			else if(len == 127) {
				if(c.inbuf.size() < 10) return;
				len = 0;
				for(int i = 0; i < 8; i++) len = (len << 8) | (uint8_t)c.inbuf[2 + i];
				hl = 10;
			}
			if(masked) hl += 4;
			if(len > 65536) { close(c.fd); c.fd = -1; return; }
			if(c.inbuf.size() < hl + len) return;
			std::string payload = c.inbuf.substr(hl, (size_t)len);
			if(masked) {
				for(size_t i = 0; i < payload.size(); i++) payload[i] ^= c.inbuf[hl - 4 + (i & 3)];
			}
			c.inbuf.erase(0, hl + (size_t)len);
			int op = b0 & 0x0F;
			if(op == 0x8) {   // close -> echo close, drop
				std::string f; f += (char)0x88; f += (char)0; send_all_nb(c.fd, f);
				close(c.fd); c.fd = -1; return;
			}
			if(op == 0x9) {   // ping -> pong
				std::string f; f += (char)0x8A; f += (char)payload.size(); f += payload;
				if(!send_all_nb(c.fd, f)) { close(c.fd); c.fd = -1; return; }
			}
		}
	}

	void loop() {
		while(!stop) {
			std::vector<pollfd> pfds;
			pfds.push_back({ listen_fd, POLLIN, 0 });
			{
				std::lock_guard<std::mutex> lk(mtx);
				for(auto &c : clients) pfds.push_back({ c.fd, POLLIN, 0 });
			}
			int r = poll(pfds.data(), pfds.size(), 200);
			if(r <= 0) continue;
			if(pfds[0].revents & POLLIN) {
				int fd = accept(listen_fd, nullptr, nullptr);
				if(fd >= 0) {
					int one = 1;
					setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
					fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
					std::lock_guard<std::mutex> lk(mtx);
					if(clients.size() < 16) clients.push_back({ fd, false, std::string() });
					else close(fd);
				}
			}
			std::lock_guard<std::mutex> lk(mtx);
			for(size_t i = 1; i < pfds.size() && i - 1 < clients.size(); i++) {
				ws_client &c = clients[i - 1];
				if(c.fd != pfds[i].fd) continue;
				if(pfds[i].revents & (POLLIN | POLLHUP | POLLERR)) {
					char buf[4096];
					ssize_t n = recv(c.fd, buf, sizeof(buf), MSG_DONTWAIT);
					if(n <= 0) {
						if(n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) { close(c.fd); c.fd = -1; }
						continue;
					}
					c.inbuf.append(buf, (size_t)n);
					if(!c.upgraded) handshake(c);
					if(c.fd >= 0 && c.upgraded) consume(c);
				}
			}
			for(size_t i = 0; i < clients.size();) {
				if(clients[i].fd < 0) clients.erase(clients.begin() + (long)i);
				else i++;
			}
		}
	}
};

ws_server::ws_server(int port, const std::string &bind_addr) : p_(new impl) {
	p_->port = port;
	int fd = socket(AF_INET, SOCK_STREAM, 0);
	if(fd < 0) return;
	int one = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	sockaddr_in sa{};
	sa.sin_family = AF_INET;
	sa.sin_port = htons((uint16_t)port);
	if(inet_pton(AF_INET, bind_addr.c_str(), &sa.sin_addr) != 1) sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if(bind(fd, (sockaddr *)&sa, sizeof(sa)) < 0 || listen(fd, 8) < 0) {
		fprintf(stderr, "elrsd: ws: bind %s:%d failed: %s\n", bind_addr.c_str(), port, strerror(errno));
		close(fd);
		return;
	}
	if(port == 0) {   // ephemeral (selftest): report the real port through port()
		socklen_t sl = sizeof(sa);
		if(getsockname(fd, (sockaddr *)&sa, &sl) == 0) p_->port = ntohs(sa.sin_port);
	}
	p_->listen_fd = fd;
	p_->thread = std::thread([this]() { p_->loop(); });
}

ws_server::~ws_server() {
	p_->stop = true;
	if(p_->thread.joinable()) p_->thread.join();
	if(p_->listen_fd >= 0) close(p_->listen_fd);
	for(auto &c : p_->clients) if(c.fd >= 0) close(c.fd);
	delete p_;
}

bool ws_server::ok() const { return p_->listen_fd >= 0; }
int ws_server::port() const { return p_->port; }

void ws_server::broadcast(const std::string &json) {
	if(!ok()) return;
	std::string f = impl::frame(json);
	std::lock_guard<std::mutex> lk(p_->mtx);
	for(auto &c : p_->clients) {
		if(c.fd < 0 || !c.upgraded) continue;
		if(!impl::send_all_nb(c.fd, f)) { close(c.fd); c.fd = -1; }
	}
}

size_t ws_server::clients() const {
	std::lock_guard<std::mutex> lk(p_->mtx);
	size_t n = 0;
	for(auto &c : p_->clients) if(c.fd >= 0 && c.upgraded) n++;
	return n;
}

} // namespace elrsd
