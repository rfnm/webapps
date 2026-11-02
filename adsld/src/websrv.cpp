// adsld: websrv - see websrv.h. Request loop lifted from common/rfnm_httpd.cpp (the
// common server) with the one addition that server cannot make: static files
// with per-extension content types.
#include "websrv.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

#include "rfnm_json.h"

namespace adsld {

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

static const char *ext_ctype(const std::string &path) {
	size_t dot = path.rfind('.');
	if(dot == std::string::npos) {
		return "application/octet-stream";
	}
	std::string e = path.substr(dot + 1);
	if(e == "html" || e == "htm") { return "text/html; charset=utf-8"; }
	if(e == "js") { return "application/javascript; charset=utf-8"; }
	if(e == "css") { return "text/css; charset=utf-8"; }
	if(e == "svg") { return "image/svg+xml"; }
	if(e == "png") { return "image/png"; }
	if(e == "json") { return "application/json"; }
	if(e == "ico") { return "image/x-icon"; }
	if(e == "woff2") { return "font/woff2"; }
	if(e == "txt" || e == "md") { return "text/plain; charset=utf-8"; }
	return "application/octet-stream";
}

bool websrv::serve_file(const std::string &root, const std::string &path,
                        std::string &body, std::string &ctype) {
	if(root.empty()) {
		return false;
	}
	std::string p = (path.empty() || path == "/") ? "/index.html" : path;
	// traversal-safe: absolute-under-root only, no ".." anywhere, no NUL/backslash tricks
	if(p[0] != '/' || p.find("..") != std::string::npos ||
	   p.find('\\') != std::string::npos || p.find('\0') != std::string::npos) {
		return false;
	}
	std::string full = root + p;
	struct stat st;
	if(stat(full.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
		return false;
	}
	FILE *f = fopen(full.c_str(), "rb");
	if(!f) {
		return false;
	}
	body.resize((size_t)st.st_size);
	size_t got = st.st_size ? fread(&body[0], 1, body.size(), f) : 0;
	fclose(f);
	if(got != body.size()) {
		return false;
	}
	ctype = ext_ctype(p);
	return true;
}

websrv::websrv(int port, std::string app_name, int api_version,
               std::vector<rfnm::endpoint_desc> endpoints,
               std::function<rfnm::http_resp(const rfnm::http_req &)> handler,
               std::string wwwroot, size_t body_cap, std::string bind_addr)
	: handler_(std::move(handler)), wwwroot_(std::move(wwwroot)), body_cap_(body_cap) {
	{
		std::string j = "{";
		bool f = true;
		rfnm::jkv(j, "app", app_name, f);
		rfnm::jkv_raw(j, "api_version", std::to_string(api_version), f);
		std::string es = "[{\"method\":\"GET\",\"path\":\"/api\",\"desc\":\"this self-description\"}";
		for(const auto &e : endpoints) {
			es += ",{\"method\":\"" + rfnm::jesc(e.method) + "\",\"path\":\"" + rfnm::jesc(e.path) +
			      "\",\"desc\":\"" + rfnm::jesc(e.desc) + "\"}";
		}
		rfnm::jkv_raw(j, "endpoints", es + "]", f);
		api_json_ = j + "}";
	}
	listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
	int one = 1;
	setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	// default is loopback (nginx publishes); a host without a front proxy can opt into LAN
	// serving via --bind 0.0.0.0
	if(inet_pton(AF_INET, bind_addr.c_str(), &addr.sin_addr) != 1) {
		fprintf(stderr, "adsld: bad bind address '%s', using 127.0.0.1\n", bind_addr.c_str());
		addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	}
	addr.sin_port = htons((uint16_t)port);
	if(bind(listen_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(listen_fd_, 8) < 0) {
		perror("adsld websrv bind/listen");
		close(listen_fd_);
		listen_fd_ = -1;
		return;
	}
	thread_ = std::thread([this] { loop(); });
}

websrv::~websrv() {
	stop_ = true;
	if(listen_fd_ >= 0) {
		shutdown(listen_fd_, SHUT_RDWR);
		close(listen_fd_);
	}
	if(thread_.joinable()) {
		thread_.join();
	}
}

void websrv::loop() {
	while(!stop_) {
		int fd = accept(listen_fd_, nullptr, nullptr);
		if(fd < 0) {
			if(stop_) {
				break;
			}
			continue;
		}
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
		rfnm::http_resp resp;
		std::string ctype = "application/json";
		if(clen > body_cap_) {
			resp = { 400, "{\"err\":\"body too large\"}" };
		} else {
			rfnm::http_req r;
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
				// unclaimed GETs fall through to the wwwroot ("/" -> index.html)
				if(resp.code == 0 && r.method == "GET" &&
				   serve_file(wwwroot_, r.path, resp.body, ctype)) {
					resp.code = 200;
				}
			}
		}
		if(resp.code == 0 || (resp.body.empty() && resp.code != 200)) {
			resp = { 404, "{\"error\":\"not found\"}" };   // same envelope as rfnm_httpd
			ctype = "application/json";
		}
		char hdr[256];
		snprintf(hdr, sizeof(hdr),
		         "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\n"
		         "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
		         resp.code, resp.code == 200 ? "OK" : "ERR", ctype.c_str(), resp.body.size());
		if(send_all(fd, hdr, strlen(hdr))) {
			send_all(fd, resp.body.data(), resp.body.size());
		}
		close(fd);
	}
}

} // namespace adsld
