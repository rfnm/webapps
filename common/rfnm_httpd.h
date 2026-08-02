// rfnm_httpd - THE fleet daemon HTTP server (loopback JSON APIs behind nginx /api/app/<id>/).
// Single canonical source replacing the per-app httpd.cpp copies (aisd/lorad/cellsyncd/vrtd
// were byte-identical modulo namespace). One accept thread, loopback bind, 500 ms RCVTIMEO,
// GET/POST/PUT with Content-Length bodies (64 KB cap), Connection: close.
//
// nginx's trailing-slash proxy_pass already strips the route prefix - handlers see bare
// paths ("/status"); the old per-app self-strips are dead. Direct loopback curls use bare
// paths too.
//
// GET /api is served BY THE HELPER from the endpoints table (uniform self-description,
// webgui/API.md section 3): {"app","api_version","endpoints":[{"method","path","desc"}]}.
#pragma once

#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace rfnm {

struct http_req {
	std::string method;   // GET | POST | PUT
	std::string path;     // no query string
	std::string query;    // raw, after '?'
	std::string body;     // POST/PUT payload (Content-Length, capped)
};

struct http_resp {
	int code = 0;         // 0 -> 404 {"error":"not found"} (the historical envelope)
	std::string body;     // JSON
};

struct endpoint_desc {
	const char *method;
	const char *path;
	const char *desc;
};

class httpd {
public:
	// app_name/api_version/endpoints feed GET /api; endpoints lists the app's own routes
	// (the /api route itself is implicit). handler sees everything except /api.
	httpd(int port, std::string app_name, int api_version, std::vector<endpoint_desc> endpoints,
	      std::function<http_resp(const http_req &)> handler, size_t body_cap = 65536);
	~httpd();
	httpd(const httpd &) = delete;
	httpd &operator=(const httpd &) = delete;
	bool ok() const { return listen_fd_ >= 0; }

private:
	void loop();
	std::function<http_resp(const http_req &)> handler_;
	std::string api_json_;
	size_t body_cap_;
	int listen_fd_ = -1;
	volatile bool stop_ = false;
	std::thread thread_;
};

} // namespace rfnm
