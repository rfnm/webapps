#include "rfnm_httpd.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

#include "rfnm_json.h"

namespace rfnm {

static bool send_all(int fd, const char *p, size_t n) {
	while(n) {
		ssize_t w = send(fd, p, n, MSG_NOSIGNAL);
		if(w <= 0) {
			return false;
		}
		p += w;
		n -= (size_t)w;
	}
	return true;
}

httpd::httpd(int port, std::string app_name, int api_version, std::vector<endpoint_desc> endpoints,
             std::function<http_resp(const http_req &)> handler, size_t body_cap)
	: handler_(std::move(handler)), body_cap_(body_cap) {
	{
		std::string j = "{";
		bool f = true;
		jkv(j, "app", app_name, f);
		jkv_raw(j, "api_version", std::to_string(api_version), f);
		std::string es = "[{\"method\":\"GET\",\"path\":\"/api\",\"desc\":\"this self-description\"}";
		for(const auto &e : endpoints) {
			es += ",{\"method\":\"" + jesc(e.method) + "\",\"path\":\"" + jesc(e.path) + "\",\"desc\":\"" + jesc(e.desc) + "\"}";
		}
		jkv_raw(j, "endpoints", es + "]", f);
		api_json_ = j + "}";
	}
	listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
	int one = 1;
	setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = htons((uint16_t)port);
	if(bind(listen_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(listen_fd_, 8) < 0) {
		perror("rfnm_httpd bind/listen");
		close(listen_fd_);
		listen_fd_ = -1;
		return;
	}
	thread_ = std::thread([this] { loop(); });
}

httpd::~httpd() {
	stop_ = true;
	if(listen_fd_ >= 0) {
		shutdown(listen_fd_, SHUT_RDWR);
		close(listen_fd_);
	}
	if(thread_.joinable()) {
		thread_.join();
	}
}

void httpd::loop() {
	while(!stop_) {
		int fd = accept(listen_fd_, nullptr, nullptr);
		if(fd < 0) {
			if(stop_) {
				break;
			}
			continue;
		}
		// bound how long one client can hold the single-threaded loop; nginx (the only
		// expected client, on loopback) delivers promptly
		struct timeval tv = { 0, 500000 };
		setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
		std::string req;
		char buf[2048];
		size_t hdr_end = std::string::npos;
		while(req.size() < 16384) {
			ssize_t r = recv(fd, buf, sizeof(buf), 0);
			if(r <= 0) {
				break;
			}
			req.append(buf, (size_t)r);
			hdr_end = req.find("\r\n\r\n");
			if(hdr_end != std::string::npos) {
				break;
			}
		}
		if(hdr_end == std::string::npos) {
			close(fd);
			continue;
		}
		char method[8] = { 0 }, target[512] = { 0 };
		sscanf(req.c_str(), "%7s %511s", method, target);
		size_t clen = 0;
		{
			std::string hdrs = req.substr(0, hdr_end);
			for(auto &ch : hdrs) {
				ch = (char)tolower((unsigned char)ch);
			}
			size_t cl = hdrs.find("content-length:");
			if(cl != std::string::npos) {
				clen = (size_t)strtoul(hdrs.c_str() + cl + 15, nullptr, 10);
			}
		}
		http_resp resp;
		if(clen > body_cap_) {
			resp = { 400, "{\"err\":\"body too large\"}" };
		} else {
			http_req r;
			r.method = method;
			std::string t = target;
			size_t qm = t.find('?');
			r.path = qm == std::string::npos ? t : t.substr(0, qm);
			r.query = qm == std::string::npos ? "" : t.substr(qm + 1);
			r.body = req.substr(hdr_end + 4);
			while(r.body.size() < clen) {
				ssize_t n = recv(fd, buf, sizeof(buf), 0);
				if(n <= 0) {
					break;
				}
				r.body.append(buf, (size_t)n);
			}
			if(r.method == "GET" && r.path == "/api") {
				resp = { 200, api_json_ };
			} else {
				resp = handler_(r);
			}
		}
		if(resp.code == 0 || resp.body.empty()) {
			resp = { 404, "{\"error\":\"not found\"}" };   // the historical fleet envelope
		}
		char hdr[256];
		snprintf(hdr, sizeof(hdr),
		         "HTTP/1.1 %d %s\r\nContent-Type: application/json\r\nContent-Length: %zu\r\n"
		         "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
		         resp.code, resp.code == 200 ? "OK" : "ERR", resp.body.size());
		if(send_all(fd, hdr, strlen(hdr))) {
			send_all(fd, resp.body.data(), resp.body.size());
		}
		close(fd);
	}
}

} // namespace rfnm
