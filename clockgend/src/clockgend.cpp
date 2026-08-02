// clockgend: Si5510 OUT9 signal-generator control daemon (the REF CLK OUT connector).
// The kernel driver owns everything hard (NA FOTF plan generation, HOST_LOAD, OUTPUT_ENABLE);
// this daemon is a thin, validated HTTP face over sysfs:
//   /sys/bus/i2c/devices/0-0058/rfnm_set_siggen_freq   [RW]
//   write kHz 1000..200099 -> retune + enable OUT9; write 0 -> disable; read -> last-set kHz.
//   /sys/bus/i2c/devices/0-0058/rfnm_ext_ref_in        [RO]  1 = a good external 10 MHz on the
//   REF CLK IN connector is disciplining the board (input selection Automatic + auto-revertive,
//   IN0 priority over the onboard IN2 TCXO), 0 = absent/bad.
//   /sys/bus/i2c/devices/0-0058/rfnm_ref_locked        [RO]  reference-PLL lock (Device-API 0x16).
// The two RO attrs cost ~6 ms of i2c traffic per read, so /status serves them through a small
// TTL cache; boards whose kernel lacks them (Lime today) report null, never an error.
// rfnm_input_status is deliberately NEVER read here: its LOS/OOF flags are latch-and-clear-on-
// read - polling it would destroy the diagnostic history for every other reader.
// Retunes are hardware-proven safe during radio streams on the rev4 base,
// so this app is radio:"none" in the manifest - it never takes the radio lease.
//
// API (JSON over HTTP, 127.0.0.1:7690 behind nginx /clkapi/):
//   GET  /status  -> {"enabled":bool,"freq_khz":N,"freq_min_khz":1000,"freq_max_khz":200099,
//                     "ext_ref":0|1|null,"ref_locked":0|1|null,"sysfs_ok":bool}
//                    (null = attr missing/unreadable on this board's kernel)
//   POST /set     -> {"freq_khz":N} enable+tune; {"freq_khz":0} or {"enabled":false} disable
//
// Selftest (no hardware): RFNM_SIGGEN_SYSFS / RFNM_EXTREF_SYSFS / RFNM_REFLOCK_SYSFS point the
// sysfs paths at temp files.
//
// Build host: make          Cross: see Makefile header (cross-compile-imx8mp skill).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

static const int FREQ_MIN_KHZ = 1000;      // 1 MHz  (SI5510_FOTF_NA_MIN_KHZ)
static const int FREQ_MAX_KHZ = 200099;    // 200.099 MHz (SI5510_FOTF_NA_MAX_KHZ)

static std::string sysfs_path() {
	const char *env = getenv("RFNM_SIGGEN_SYSFS");
	if(env && *env) {
		return env;
	}
	const char *cands[] = {
		"/sys/bus/i2c/devices/0-0058/rfnm_set_siggen_freq",
		"/sys/devices/platform/soc@0/30800000.bus/30a20000.i2c/i2c-0/0-0058/rfnm_set_siggen_freq",
	};
	for(const char *c : cands) {
		std::ifstream f(c);
		if(f) {
			return c;
		}
	}
	return cands[0];   // report the canonical path in errors even if absent
}

static bool siggen_read(int &khz) {
	std::ifstream f(sysfs_path());
	if(!f) {
		return false;
	}
	f >> khz;
	return !f.fail();
}

static bool siggen_write(int khz) {
	std::ofstream f(sysfs_path());
	if(!f) {
		return false;
	}
	f << khz << "\n";
	f.close();
	return !f.fail();
}

// khz==0 is always valid (disable); anything else must sit on the NA grid
static bool freq_valid(long khz) {
	return khz == 0 || (khz >= FREQ_MIN_KHZ && khz <= FREQ_MAX_KHZ);
}

// ---- reference-status attrs (ext-ref presence + reference-PLL lock), TTL-cached ----------
static const long REF_CACHE_TTL_MS = 2500;   // each sysfs read is ~6 ms of i2c traffic; GUI polls /status every second

struct ref_attr {
	const char *env;      // selftest override, RFNM_SIGGEN_SYSFS-style
	const char *attr;     // sysfs attribute name under the si5510 device node
	long stamp_ms = 0;    // CLOCK_MONOTONIC of the last real read; 0 = never read
	int val = -1;         // cached value: 0/1, or -1 = missing/unreadable/garbage (JSON null)
};
static ref_attr g_ext_ref = { "RFNM_EXTREF_SYSFS", "rfnm_ext_ref_in" };
static ref_attr g_ref_locked = { "RFNM_REFLOCK_SYSFS", "rfnm_ref_locked" };

static long mono_ms() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static std::string ref_attr_path(const ref_attr &a) {
	const char *env = getenv(a.env);
	if(env && *env) {
		return env;
	}
	const std::string dirs[] = {
		"/sys/bus/i2c/devices/0-0058/",
		"/sys/devices/platform/soc@0/30800000.bus/30a20000.i2c/i2c-0/0-0058/",
	};
	for(const std::string &d : dirs) {
		std::ifstream f(d + a.attr);
		if(f) {
			return d + a.attr;
		}
	}
	return dirs[0] + a.attr;   // report the canonical path in errors even if absent
}

// lazy cached read of one 0/1 flag; anything not exactly 0 or 1 (absent file, garbage) -> -1
static int ref_flag(ref_attr &a) {
	long now = mono_ms();
	if(a.stamp_ms && now - a.stamp_ms < REF_CACHE_TTL_MS) {
		return a.val;
	}
	a.stamp_ms = now;
	a.val = -1;
	std::ifstream f(ref_attr_path(a));
	if(f) {
		int v = -1;
		f >> v;
		if(!f.fail() && (v == 0 || v == 1)) {
			a.val = v;
		}
	}
	return a.val;
}

static const char *flag_json(int v) {
	return v < 0 ? "null" : v ? "1" : "0";
}

static std::string status_json() {
	int khz = 0;
	bool have = siggen_read(khz);
	char buf[256];
	snprintf(buf, sizeof(buf),
	         "{\"enabled\":%s,\"freq_khz\":%d,\"freq_min_khz\":%d,\"freq_max_khz\":%d,\"ext_ref\":%s,\"ref_locked\":%s,\"sysfs_ok\":%s}",
	         (have && khz > 0) ? "true" : "false", have ? khz : 0, FREQ_MIN_KHZ, FREQ_MAX_KHZ,
	         flag_json(ref_flag(g_ext_ref)), flag_json(ref_flag(g_ref_locked)), have ? "true" : "false");
	return buf;
}

// tiny single-field JSON number/bool extractor - bodies here are {"freq_khz":N} / {"enabled":false}
static bool json_num(const std::string &body, const char *key, long &out) {
	size_t k = body.find(std::string("\"") + key + "\"");
	if(k == std::string::npos) {
		return false;
	}
	size_t c = body.find(':', k);
	if(c == std::string::npos) {
		return false;
	}
	char *end = nullptr;
	long v = strtol(body.c_str() + c + 1, &end, 10);
	if(end == body.c_str() + c + 1) {
		return false;
	}
	out = v;
	return true;
}

static bool json_false(const std::string &body, const char *key) {
	size_t k = body.find(std::string("\"") + key + "\"");
	if(k == std::string::npos) {
		return false;
	}
	size_t c = body.find(':', k);
	return c != std::string::npos && body.compare(c + 1, 5, "false") == 0;
}

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

static void reply(int fd, int code, const std::string &body) {
	const char *msg = code == 200 ? "OK" : code == 400 ? "Bad Request" : code == 404 ? "Not Found" : "Internal Server Error";
	char hdr[192];
	snprintf(hdr, sizeof(hdr), "HTTP/1.1 %d %s\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nContent-Length: %zu\r\nConnection: close\r\n\r\n",
	         code, msg, body.size());
	if(send_all(fd, hdr, strlen(hdr))) {
		send_all(fd, body.data(), body.size());
	}
}

static void handle(int fd) {
	struct timeval tv = { 2, 0 };
	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	std::string req;
	char buf[2048];
	size_t hdr_end = std::string::npos;
	while(req.size() < 8192) {
		ssize_t n = recv(fd, buf, sizeof(buf), 0);
		if(n <= 0) {
			break;
		}
		req.append(buf, (size_t)n);
		hdr_end = req.find("\r\n\r\n");
		if(hdr_end != std::string::npos) {
			break;
		}
	}
	if(hdr_end == std::string::npos) {
		close(fd);
		return;
	}
	char method[8] = { 0 }, path[128] = { 0 };
	sscanf(req.c_str(), "%7s %127s", method, path);
	size_t clen = 0;
	{
		std::string h = req.substr(0, hdr_end);
		for(auto &ch : h) {
			ch = (char)tolower((unsigned char)ch);
		}
		size_t cl = h.find("content-length:");
		if(cl != std::string::npos) {
			clen = (size_t)strtoul(h.c_str() + cl + 15, nullptr, 10);
		}
	}
	std::string body = req.substr(hdr_end + 4);
	while(body.size() < clen && clen < 4096) {
		ssize_t n = recv(fd, buf, sizeof(buf), 0);
		if(n <= 0) {
			break;
		}
		body.append(buf, (size_t)n);
	}

	if(strcmp(method, "GET") == 0 && strcmp(path, "/status") == 0) {
		reply(fd, 200, status_json());
	} else if((strcmp(method, "POST") == 0 || strcmp(method, "PUT") == 0) && strcmp(path, "/set") == 0) {
		long khz = -1;
		if(json_false(body, "enabled")) {
			khz = 0;
		} else if(!json_num(body, "freq_khz", khz)) {
			reply(fd, 400, "{\"error\":\"need freq_khz (kHz) or enabled:false\"}");
			close(fd);
			return;
		}
		if(!freq_valid(khz)) {
			reply(fd, 400, "{\"error\":\"freq_khz must be 0 (off) or 1000..200099\"}");
			close(fd);
			return;
		}
		if(!siggen_write((int)khz)) {
			reply(fd, 500, "{\"error\":\"sysfs write failed (driver missing?)\"}");
			close(fd);
			return;
		}
		printf("clockgend: OUT9 %s\n", khz ? (std::to_string(khz) + " kHz").c_str() : "off");
		fflush(stdout);
		reply(fd, 200, status_json());
	} else {
		reply(fd, 404, "{\"error\":\"not found\"}");
	}
	close(fd);
}

// ---- selftest (hardware-free: sysfs redirected to a temp file) ---------------------------
static int fails = 0;
static void check(bool ok, const char *name) {
	printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
	if(!ok) {
		fails++;
	}
}

static void put_file(const char *path, const char *text) {
	std::ofstream f(path);
	f << text;
}

static void ref_cache_reset() {
	g_ext_ref.stamp_ms = 0;
	g_ref_locked.stamp_ms = 0;
}

static int selftest() {
	char tmpl[] = "/tmp/clockgend-selftest-XXXXXX";
	char eref[] = "/tmp/clockgend-selftest-eref-XXXXXX";
	char lock[] = "/tmp/clockgend-selftest-lock-XXXXXX";
	for(char *t : { tmpl, eref, lock }) {
		int tfd = mkstemp(t);
		if(tfd < 0) {
			fprintf(stderr, "selftest: mkstemp failed\n");
			return 1;
		}
		close(tfd);
	}
	setenv("RFNM_SIGGEN_SYSFS", tmpl, 1);
	setenv("RFNM_EXTREF_SYSFS", eref, 1);
	setenv("RFNM_REFLOCK_SYSFS", lock, 1);

	check(freq_valid(0), "valid_off");
	check(freq_valid(1000) && freq_valid(200099) && freq_valid(10000), "valid_range");
	check(!freq_valid(999) && !freq_valid(200100) && !freq_valid(-5), "invalid_rejected");

	int khz = -1;
	check(siggen_write(10000) && siggen_read(khz) && khz == 10000, "write_readback_10MHz");
	check(siggen_write(0) && siggen_read(khz) && khz == 0, "write_readback_off");

	long v = 0;
	check(json_num("{\"freq_khz\":12345}", "freq_khz", v) && v == 12345, "json_num");
	check(json_false("{\"enabled\":false}", "enabled"), "json_false");
	check(!json_num("{\"freq\":1}", "freq_khz", v), "json_missing_key");

	siggen_write(155520);
	std::string s = status_json();
	check(s.find("\"enabled\":true") != std::string::npos && s.find("\"freq_khz\":155520") != std::string::npos, "status_json_on");

	// reference-status attrs: present (0/1), cache TTL, garbage, absent -> null
	put_file(eref, "1\n");
	put_file(lock, "1\n");
	ref_cache_reset();
	s = status_json();
	check(s.find("\"ext_ref\":1") != std::string::npos && s.find("\"ref_locked\":1") != std::string::npos, "ref_present_1");
	put_file(eref, "0\n");
	ref_cache_reset();
	check(status_json().find("\"ext_ref\":0") != std::string::npos, "ref_present_0");
	put_file(eref, "1\n");   // fresh sysfs value must NOT surface until the TTL lapses
	check(status_json().find("\"ext_ref\":0") != std::string::npos, "ref_cache_holds");
	ref_cache_reset();
	check(status_json().find("\"ext_ref\":1") != std::string::npos, "ref_cache_refresh");
	put_file(eref, "bogus\n");
	put_file(lock, "7\n");   // not a 0/1 flag -> null, never a lie
	ref_cache_reset();
	s = status_json();
	check(s.find("\"ext_ref\":null") != std::string::npos && s.find("\"ref_locked\":null") != std::string::npos, "ref_garbage_null");
	unlink(eref);
	unlink(lock);
	ref_cache_reset();
	s = status_json();
	check(s.find("\"ext_ref\":null") != std::string::npos && s.find("\"ref_locked\":null") != std::string::npos, "ref_absent_null");

	unlink(tmpl);
	printf(fails ? "SELFTEST FAILED (%d)\n" : "SELFTEST OK\n", fails);
	return fails ? 1 : 0;
}

int main(int argc, char **argv) {
	if(argc > 1 && strcmp(argv[1], "--selftest") == 0) {
		return selftest();
	}
	int port = 7690;
	for(int i = 1; i < argc - 1; i++) {
		if(strcmp(argv[i], "--port") == 0) {
			port = atoi(argv[i + 1]);
		}
	}
	signal(SIGPIPE, SIG_IGN);

	int lfd = socket(AF_INET, SOCK_STREAM, 0);
	int one = 1;
	setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = htons((uint16_t)port);
	if(bind(lfd, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(lfd, 8) < 0) {
		perror("clockgend: bind/listen");
		return 1;
	}
	int khz = -1;
	printf("clockgend: sysfs %s (%s), ref attrs %s, listening on 127.0.0.1:%d\n",
	       sysfs_path().c_str(), siggen_read(khz) ? "ok" : "MISSING",
	       ref_flag(g_ext_ref) < 0 ? "absent (ext_ref/ref_locked -> null)" : "ok", port);
	fflush(stdout);
	while(true) {
		int fd = accept(lfd, nullptr, nullptr);
		if(fd >= 0) {
			handle(fd);   // single-threaded: nginx on loopback is the only client
		}
	}
}
