// iod: RFNM Board I/O daemon - the motherboard bottom-edge header, phase 1 (GPIO only).
// Talks the kernel GPIO chardev uapi v2 directly (/dev/gpiochipN + ioctls): no libgpiod
// dependency (the rootfs ships only the runtime .so), static-friendly, holds requested
// lines so output values persist while the daemon runs (released lines revert to hi-Z).
//
// Pin table = the works-today header pins (render-confirmed only).
// GUARDED pins (per the /rfnm/board.conf IO_GUARD list - lines reserved by board policy,
// e.g. wired to external RF hardware) are rendered read-only: iod never requests them as
// outputs, and /set refuses them - flipping such a line from a web page is unsafe.
// PWM3 / GPT1 / PPS pins are visualization-only placeholders until their DT bits land.
//
// API (JSON over HTTP, 127.0.0.1:7691 behind nginx /ioapi/):
//   GET  /status -> {"pins":[{id,chip,line,label,silk,dir,val,guarded,planned}...]}
//   POST /set    -> {"id":"gpio1_10","dir":"out","val":1} | {"id":..,"dir":"in"}
// Selftest: RFNM_IOD_MOCK=1 skips the chardev (table/JSON/parse checks only).
//
// Build host: make. Cross: cross-compile-imx8mp skill env (no deps beyond kernel uapi).

#include <fcntl.h>
#include <linux/gpio.h>
#include <netinet/in.h>
#include <signal.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <string>

struct pin_def {
	const char *id;
	int chip;              // /dev/gpiochip<chip> for the pad's ALT5 GPIO; chipN = SoC GPIO(N+1)
	int line;
	const char *label;     // pad/function note
	const char *silk;      // board silk text
	bool guarded;          // never driven by iod (reserved by board policy)
	bool planned;          // visualization-only, no chardev/mux access
	// runtime pad-mux (direct IOMUXC write, DT does not own these pads).
	// mux_off 0 = not muxable from iod (readback still done when nonzero).
	uint32_t mux_off;      // IOMUXC MUX_CTL offset (base 0x30330000)
	int alt_primary;       // the pad's named function (silk) ALT value, -1 = none usable yet
	const char *primary;   // name of that function
	bool muxable;          // user may switch primary <-> GPIO(ALT5)
};

// chip numbering: gpiochipN = GPIO(N+1); e.g. GPIO4_IO26 = chip 3 line 26
// ONLY render-confirmed header pins. A board reset during the first iod
// verify happened right around an output test on GPIO1_IO10 (a pin from a COMMENTED-OUT
// DTS block, header routing unconfirmed) - cause unattributable (volatile journal, and
// that board had a known spontaneous-reboot issue), so the table retreats to pins whose
// silk is on the board render; PPS_IN-select pins are INPUT-ONLY until the schematic
// confirms their nets. Guarded here means the pin is reserved by board policy.
// mux offsets from imx8mp-pinfunc.h (validated against absolute IOMUXC addresses).
// GPIO alternates are the pads' ALT5 per the i.MX8MP convention. pwm3 primary = ALT1
// PWM3_OUT (live since the 2026-07-10 DT); gpt1/i2c4 primaries have no kernel consumer yet,
// so their GPIO alternate is freely selectable (the "grey pinmux alternative").
static pin_def PINS[] = {
	{ "gpio4_26",  3, 26, "SAI2_TXD0 as GPIO4_IO26",  "GPIO4 26",   true,  false, 0x1B0, -1, "reserved", false },
	{ "gpio4_30",  3, 30, "SAI3_RXD as GPIO4_IO30",   "GPIO4 30",   true,  false, 0x1C0, -1, "reserved", false },
	{ "gpio4_27",  3, 27, "SAI2_MCLK (PPS_IN sel A) - input-only", "PPS IN", true, false, 0x1B4, -1, "pps-in", false },
	{ "gpio3_19",  2, 19, "SAI5_RXFS (PPS_IN sel B) - input-only", "PPS IN B", true, false, 0x12C, -1, "pps-in", false },
	{ "pwm3",      4,  3, "SPDIF_TX: PWM3_OUT / GPIO5_IO03", "PWM3", false, false, 0x1D4, 1, "PWM3", true },
	{ "gpt1_capt1",4,  0, "SAI3_TXC: GPT1_CAPTURE1 / GPIO5_IO00", "GPT1 CAPT1", false, false, 0x1C8, -1, "GPT1 CAPT1 (module pending)", true },
	{ "gpt1_cmp3", 4,  5, "SPDIF_EXT_CLK = ETH PHY reset on this board (DTS) - the real CMP3 header pad needs the schematic", "GPT1 CMP3", true, false, 0x1DC, -1, "ETH PHY reset", false },
	{ "i2c4_sda",  4, 21, "I2C4_SDA: I2C4 / GPIO5_IO21", "I2C4 SDA", false, false, 0x21C, -1, "I2C4 (DT pending)", true },
	{ "i2c4_scl",  4, 20, "I2C4_SCL: I2C4 / GPIO5_IO20", "I2C4 SCL", false, false, 0x218, -1, "I2C4 (DT pending)", true },
	{ "pps_out",  -1, -1, "source TBD (Si5510 vs SoC)", "PPS OUT",  false, true, 0, -1, "", false },
	// UART pads: uart4 is disabled in the DTS (no consumer - a test script already
	// toggles its TX as GPIO5_29), so both pads are freely muxable UART4 <-> GPIO. uart2 IS
	// the Linux console (console=ttymxc1) - readback only, never muxed away from here.
	{ "uart4_tx",  4, 29, "UART4_TXD: UART4 / GPIO5_IO29", "UART4 TX", false, false, 0x23C, 0, "UART4 (unused)", true },
	{ "uart4_rx",  4, 28, "UART4_RXD: UART4 / GPIO5_IO28", "UART4 RX", false, false, 0x238, 0, "UART4 (unused)", true },
	// SURPRISE: the Linux console runs on SD1_DATA2/3 pads (pinctrl_uart2), so the
	// dedicated UART2 pads on the header are FREE GPIOs. No UART2 alternate offered - a
	// second pad pair on the console controller would conflict.
	{ "uart2_tx",  4, 25, "UART2_TXD pad - free GPIO5_IO25 (console lives on SD1 pads)", "UART2 TX", false, false, 0x22C, -1, "GPIO", false },
	{ "uart2_rx",  4, 24, "UART2_RXD pad - free GPIO5_IO24 (console lives on SD1 pads)", "UART2 RX", false, false, 0x228, -1, "GPIO", false },
};
static const int NPINS = sizeof(PINS) / sizeof(PINS[0]);

struct pin_state {
	int fd = -1;           // held line fd (v2 line request), -1 = not requested
	bool is_out = false;
	int val = -1;          // last read/driven value, -1 unknown
};
static pin_state g_state[NPINS];
static bool g_mock = false;

// ---- PWM3 (i.MX pwm3 @ 30680000, enabled in DT 2026-07-10; /sys/class/pwm) ---------------
static std::string g_pwm;   // .../pwmchipN with device -> 30680000.pwm; "" = absent

static std::string file_read(const std::string &path) {
	FILE *f = fopen(path.c_str(), "r");
	if(!f) {
		return "";
	}
	char buf[64] = { 0 };
	if(!fgets(buf, sizeof(buf), f)) {
		buf[0] = 0;
	}
	fclose(f);
	std::string v = buf;
	while(!v.empty() && (v.back() == '\n' || v.back() == ' ')) {
		v.pop_back();
	}
	return v;
}

static bool file_write(const std::string &path, const std::string &v) {
	FILE *f = fopen(path.c_str(), "w");
	if(!f) {
		return false;
	}
	bool ok = fprintf(f, "%s\n", v.c_str()) > 0;
	fclose(f);
	return ok;
}

static void pwm_resolve() {
	for(int n = 0; n < 8; n++) {
		char link[128], dst[256];
		snprintf(link, sizeof(link), "/sys/class/pwm/pwmchip%d/device", n);
		ssize_t r = readlink(link, dst, sizeof(dst) - 1);
		if(r <= 0) {
			continue;
		}
		dst[r] = 0;
		if(strstr(dst, "30680000.pwm")) {
			char base[64];
			snprintf(base, sizeof(base), "/sys/class/pwm/pwmchip%d", n);
			g_pwm = base;
			if(file_read(g_pwm + "/pwm0/period").empty()) {
				file_write(g_pwm + "/export", "0");   // idempotent; EBUSY if already exported
			}
			return;
		}
	}
}

static std::string pwm_json() {
	if(g_pwm.empty()) {
		return "{\"found\":false}";
	}
	std::string en = file_read(g_pwm + "/pwm0/enable"), per = file_read(g_pwm + "/pwm0/period"), dut = file_read(g_pwm + "/pwm0/duty_cycle");
	return "{\"found\":true,\"enable\":" + (en.empty() ? "0" : en) + ",\"period_ns\":" + (per.empty() ? "0" : per) +
	       ",\"duty_ns\":" + (dut.empty() ? "0" : dut) + "}";
}

// order matters in the pwm sysfs: duty must never exceed period, so zero it first
static bool pwm_set(long period_ns, long duty_ns, long enable) {
	if(g_pwm.empty()) {
		return false;
	}
	std::string d = g_pwm + "/pwm0/";
	if(period_ns > 0) {
		file_write(d + "duty_cycle", "0");
		if(!file_write(d + "period", std::to_string(period_ns))) {
			return false;
		}
	}
	if(duty_ns >= 0 && !file_write(d + "duty_cycle", std::to_string(duty_ns))) {
		return false;
	}
	return file_write(d + "enable", enable ? "1" : "0");
}

// ---- IOMUXC runtime pad mux (mmap /dev/mem; peripheral regs, not a no-map RAM region) ----
#include <sys/mman.h>
static volatile uint32_t *g_iomuxc = nullptr;   // 4 KiB at 0x30330000

static bool iomuxc_map() {
	if(g_iomuxc || g_mock) {
		return g_iomuxc != nullptr;
	}
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if(fd < 0) {
		return false;
	}
	void *m = mmap(nullptr, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x30330000);
	close(fd);
	if(m == MAP_FAILED) {
		return false;
	}
	g_iomuxc = (volatile uint32_t *)m;
	return true;
}

static int mux_read(const pin_def &d) {
	if(!d.mux_off || !iomuxc_map()) {
		return -1;
	}
	return (int)(g_iomuxc[d.mux_off / 4] & 0x7);
}

static bool mux_write(const pin_def &d, int alt) {
	if(!d.mux_off || !iomuxc_map()) {
		return false;
	}
	g_iomuxc[d.mux_off / 4] = (uint32_t)alt;
	return true;
}

static int chip_open(int chip) {
	char path[32];
	snprintf(path, sizeof(path), "/dev/gpiochip%d", chip);
	return open(path, O_RDWR | O_CLOEXEC);
}

// (re)request one pin as input or output-with-value; holds the fd so outputs persist
static bool pin_request(int idx, bool out, int val) {
	pin_def &d = PINS[idx];
	pin_state &st = g_state[idx];
	if(d.planned || (d.guarded && out)) {
		return false;
	}
	if(st.fd >= 0) {
		close(st.fd);
		st.fd = -1;
	}
	int cfd = chip_open(d.chip);
	if(cfd < 0) {
		return false;
	}
	struct gpio_v2_line_request req;
	memset(&req, 0, sizeof(req));
	req.offsets[0] = (uint32_t)d.line;
	req.num_lines = 1;
	snprintf(req.consumer, sizeof(req.consumer), "iod");
	req.config.flags = out ? GPIO_V2_LINE_FLAG_OUTPUT : GPIO_V2_LINE_FLAG_INPUT;
	if(out) {
		req.config.num_attrs = 1;
		req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
		req.config.attrs[0].attr.values = val ? 1 : 0;
		req.config.attrs[0].mask = 1;
	}
	int r = ioctl(cfd, GPIO_V2_GET_LINE_IOCTL, &req);
	close(cfd);
	if(r < 0 || req.fd < 0) {
		return false;
	}
	st.fd = req.fd;
	st.is_out = out;
	st.val = out ? (val ? 1 : 0) : -1;
	return true;
}

static int pin_read(int idx) {
	pin_state &st = g_state[idx];
	if(st.fd < 0) {
		return -1;
	}
	struct gpio_v2_line_values lv;
	lv.mask = 1;
	lv.bits = 0;
	if(ioctl(st.fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &lv) < 0) {
		return -1;
	}
	st.val = lv.bits & 1;
	return st.val;
}

static bool pin_drive(int idx, int val) {
	pin_state &st = g_state[idx];
	if(st.fd < 0 || !st.is_out) {
		return false;
	}
	struct gpio_v2_line_values lv;
	lv.mask = 1;
	lv.bits = val ? 1 : 0;
	if(ioctl(st.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &lv) < 0) {
		return false;
	}
	st.val = val ? 1 : 0;
	return true;
}

static std::string status_json() {
	std::string j = "{\"pins\":[";
	for(int i = 0; i < NPINS; i++) {
		pin_def &d = PINS[i];
		pin_state &st = g_state[i];
		int mux = g_mock ? -1 : mux_read(d);
		bool as_gpio = mux == 5;
		// a muxable pin is only chardev-readable while its pad sits on the GPIO alternate
		int v = (!g_mock && !d.planned && (as_gpio || !d.muxable)) ? (st.is_out ? st.val : pin_read(i)) : -1;
		char buf[384];
		snprintf(buf, sizeof(buf),
		         "%s{\"id\":\"%s\",\"chip\":%d,\"line\":%d,\"label\":\"%s\",\"silk\":\"%s\","
		         "\"dir\":\"%s\",\"val\":%d,\"guarded\":%s,\"planned\":%s,"
		         "\"muxable\":%s,\"mux\":%d,\"mux_gpio\":%s,\"primary\":\"%s\",\"gpio_name\":\"GPIO%d_%02d\"}",
		         i ? "," : "", d.id, d.chip, d.line, d.label, d.silk,
		         st.is_out ? "out" : "in", v, d.guarded ? "true" : "false", d.planned ? "true" : "false",
		         d.muxable ? "true" : "false", mux, as_gpio ? "true" : "false", d.primary, d.chip + 1, d.line);
		j += buf;
	}
	return j + "],\"pwm3\":" + pwm_json() + "}";
}

static int find_pin(const std::string &id) {
	for(int i = 0; i < NPINS; i++) {
		if(id == PINS[i].id) {
			return i;
		}
	}
	return -1;
}

static std::string json_str(const std::string &body, const char *key) {
	size_t k = body.find(std::string("\"") + key + "\"");
	if(k == std::string::npos) {
		return "";
	}
	size_t c = body.find(':', k);
	size_t q1 = body.find('"', c);
	size_t q2 = q1 == std::string::npos ? std::string::npos : body.find('"', q1 + 1);
	return q2 == std::string::npos ? "" : body.substr(q1 + 1, q2 - q1 - 1);
}

static long json_num(const std::string &body, const char *key, long def) {
	size_t k = body.find(std::string("\"") + key + "\"");
	if(k == std::string::npos) {
		return def;
	}
	size_t c = body.find(':', k);
	return c == std::string::npos ? def : strtol(body.c_str() + c + 1, nullptr, 10);
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
	const char *msg = code == 200 ? "OK" : code == 400 ? "Bad Request" : code == 403 ? "Forbidden" : code == 404 ? "Not Found" : "Internal Server Error";
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
	} else if(strcmp(method, "POST") == 0 && strcmp(path, "/set") == 0) {
		int idx = find_pin(json_str(body, "id"));
		std::string dir = json_str(body, "dir");
		long val = json_num(body, "val", 0);
		if(idx < 0) {
			reply(fd, 404, "{\"error\":\"unknown pin\"}");
		} else if(PINS[idx].planned) {
			reply(fd, 400, "{\"error\":\"pin not wired yet (DT/module pending)\"}");
		} else if(PINS[idx].guarded && dir == "out") {
			reply(fd, 403, "{\"error\":\"guarded pin - read-only from iod (see the pin note: RF switch or kernel-owned)\"}");
		} else if(dir != "in" && dir != "out") {
			reply(fd, 400, "{\"error\":\"dir must be in|out\"}");
		} else if(!g_mock && PINS[idx].muxable && mux_read(PINS[idx]) != 5) {
			reply(fd, 409, "{\"error\":\"pad is on its primary function - switch it to the GPIO alternate first\"}");
		} else if(g_mock) {
			reply(fd, 200, status_json());
		} else {
			bool ok;
			if(dir == "out" && g_state[idx].is_out && g_state[idx].fd >= 0) {
				ok = pin_drive(idx, (int)val);   // fast path: already an output
			} else {
				ok = pin_request(idx, dir == "out", (int)val);
			}
			if(!ok) {
				reply(fd, 500, "{\"error\":\"gpio request/drive failed (line busy?)\"}");
			} else {
				printf("iod: %s -> %s%s\n", PINS[idx].id, dir.c_str(), dir == "out" ? (val ? " 1" : " 0") : "");
				fflush(stdout);
				reply(fd, 200, status_json());
			}
		}
	} else if(strcmp(method, "POST") == 0 && strcmp(path, "/mux") == 0) {
		int idx = find_pin(json_str(body, "id"));
		std::string to = json_str(body, "to");
		if(idx < 0 || !PINS[idx].muxable) {
			reply(fd, idx < 0 ? 404 : 400, "{\"error\":\"pin not muxable from here\"}");
		} else if(to != "gpio" && to != "primary") {
			reply(fd, 400, "{\"error\":\"to must be gpio|primary\"}");
		} else if(to == "primary" && PINS[idx].alt_primary < 0) {
			reply(fd, 400, "{\"error\":\"primary function has no kernel consumer yet - stays on GPIO or default\"}");
		} else if(g_mock) {
			reply(fd, 200, status_json());
		} else if(strcmp(PINS[idx].id, "pwm3") == 0 && to == "gpio" && file_read(g_pwm + "/pwm0/enable") == "1") {
			reply(fd, 409, "{\"error\":\"PWM3 is enabled - disable it before taking the pad as GPIO\"}");
		} else {
			int alt = to == "gpio" ? 5 : PINS[idx].alt_primary;
			if(to == "primary" && g_state[idx].fd >= 0) {
				close(g_state[idx].fd);   // release the chardev line before handing the pad back
				g_state[idx].fd = -1;
				g_state[idx].val = -1;
			}
			if(!mux_write(PINS[idx], alt)) {
				reply(fd, 500, "{\"error\":\"IOMUXC write failed (/dev/mem)\"}");
			} else {
				if(to == "gpio") {
					pin_request(idx, false, 0);   // hold as input for live view
				}
				printf("iod: %s pad -> %s (ALT%d)\n", PINS[idx].id, to.c_str(), alt);
				fflush(stdout);
				reply(fd, 200, status_json());
			}
		}
	} else if(strcmp(method, "POST") == 0 && strcmp(path, "/pwm") == 0) {
		long period = json_num(body, "period_ns", 0);
		long duty = json_num(body, "duty_ns", -1);
		long enable = json_num(body, "enable", 0);
		if(g_mock || g_pwm.empty()) {
			reply(fd, g_mock ? 200 : 404, g_mock ? status_json() : "{\"error\":\"pwm3 not present (DT/kernel too old?)\"}");
		} else if(period < 0 || duty > period) {
			reply(fd, 400, "{\"error\":\"need duty_ns <= period_ns\"}");
		} else if(!pwm_set(period, duty, enable)) {
			reply(fd, 500, "{\"error\":\"pwm sysfs write failed\"}");
		} else {
			printf("iod: pwm3 period=%ld duty=%ld enable=%ld\n", period, duty, enable);
			fflush(stdout);
			reply(fd, 200, status_json());
		}
	} else {
		reply(fd, 404, "{\"error\":\"not found\"}");
	}
	close(fd);
}

// ---- selftest (hardware-free) --------------------------------------------------------------
static int fails = 0;
static void check(bool ok, const char *name) {
	printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
	if(!ok) {
		fails++;
	}
}

static int selftest() {
	g_mock = true;
	check(find_pin("gpio4_26") == 0 && PINS[0].guarded, "guard_table");
	check(find_pin("gpio4_27") >= 0 && PINS[find_pin("gpio4_27")].guarded, "pps_sel_input_only");
	check(find_pin("gpio1_10") < 0, "unconfirmed_pins_removed");
	check(find_pin("pwm3") >= 0 && PINS[find_pin("pwm3")].muxable && PINS[find_pin("pwm3")].alt_primary == 1, "pwm3_muxable");
	check(find_pin("gpt1_cmp3") >= 0 && PINS[find_pin("gpt1_cmp3")].guarded && !PINS[find_pin("gpt1_cmp3")].muxable, "cmp3_phy_reset_guarded");
	check(find_pin("pps_out") >= 0 && PINS[find_pin("pps_out")].planned, "planned_pin");
	check(find_pin("nope") < 0, "unknown_pin");
	std::string s = status_json();
	check(s.find("\"id\":\"gpio4_26\"") != std::string::npos && s.find("\"guarded\":true") != std::string::npos, "status_shape");
	check(s.find("\"muxable\":true") != std::string::npos && s.find("\"gpio_name\":\"GPIO5_03\"") != std::string::npos, "mux_shape");
	check(json_str("{\"id\":\"gpio1_10\",\"dir\":\"out\"}", "dir") == "out", "json_str");
	check(json_num("{\"val\":1}", "val", 0) == 1, "json_num");
	check(status_json().find("\"pwm3\":{\"found\":false") != std::string::npos, "pwm_absent_shape");
	printf(fails ? "SELFTEST FAILED (%d)\n" : "SELFTEST OK\n", fails);
	return fails ? 1 : 0;
}

int main(int argc, char **argv) {
	if(argc > 1 && strcmp(argv[1], "--selftest") == 0) {
		return selftest();
	}
	g_mock = getenv("RFNM_IOD_MOCK") != nullptr;
	int port = 7691;
	for(int i = 1; i < argc - 1; i++) {
		if(strcmp(argv[i], "--port") == 0) {
			port = atoi(argv[i + 1]);
		}
	}
	signal(SIGPIPE, SIG_IGN);

	if(!g_mock) {
		pwm_resolve();
	}
	// take every real pin as input at startup (visualize); outputs happen on demand
	int held = 0;
	if(!g_mock) {
		iomuxc_map();
		for(int i = 0; i < NPINS; i++) {
			bool on_gpio = PINS[i].muxable ? mux_read(PINS[i]) == 5 : true;
			if(!PINS[i].planned && on_gpio && pin_request(i, false, 0)) {
				held++;
			}
		}
	}

	int lfd = socket(AF_INET, SOCK_STREAM, 0);
	int one = 1;
	setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = htons((uint16_t)port);
	if(bind(lfd, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(lfd, 8) < 0) {
		perror("iod: bind/listen");
		return 1;
	}
	printf("iod: %d/%d pins held as inputs, listening on 127.0.0.1:%d%s\n", held, NPINS, port, g_mock ? " (MOCK)" : "");
	fflush(stdout);
	while(true) {
		int fd = accept(lfd, nullptr, nullptr);
		if(fd >= 0) {
			handle(fd);   // single-threaded: nginx on loopback is the only client
		}
	}
}
