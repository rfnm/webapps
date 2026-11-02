// adsld: HTTP server with static-file serving for the map web UI. common/rfnm_httpd.h is
// JSON-only (hardcoded application/json content type), so this is the adsld-side extension:
// same request surface and GET /api self-description, same loopback bind + 500 ms RCVTIMEO +
// Connection: close posture, but GETs the JSON handler declines (code 0) fall through to
// files under a wwwroot with per-extension content types. "/" serves index.html; any path
// containing ".." is rejected before it reaches the filesystem. Non-GETs and misses keep the
// 404 {"error":"not found"} envelope.
#pragma once

#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "rfnm_httpd.h"   // rfnm::http_req / http_resp / endpoint_desc (types only)

namespace adsld {

class websrv {
public:
	// api handler: exactly the rfnm::httpd contract - sees everything except /api, returns
	// code 0 for "not mine". wwwroot: directory served for unclaimed GETs (empty = no
	// static serving, pure rfnm_httpd behavior).
	websrv(int port, std::string app_name, int api_version,
	       std::vector<rfnm::endpoint_desc> endpoints,
	       std::function<rfnm::http_resp(const rfnm::http_req &)> handler,
	       std::string wwwroot, size_t body_cap = 65536,
	       std::string bind_addr = "127.0.0.1");
	~websrv();
	websrv(const websrv &) = delete;
	websrv &operator=(const websrv &) = delete;
	bool ok() const { return listen_fd_ >= 0; }

	// static-file lookup (exposed for tests): resolves path under root, refuses "..",
	// fills body + content type; false = not served (caller 404s)
	static bool serve_file(const std::string &root, const std::string &path,
	                       std::string &body, std::string &ctype);

private:
	void loop();
	std::function<rfnm::http_resp(const rfnm::http_req &)> handler_;
	std::string api_json_;
	std::string wwwroot_;
	size_t body_cap_;
	int listen_fd_ = -1;
	volatile bool stop_ = false;
	std::thread thread_;
};

} // namespace adsld
