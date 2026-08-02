// rfnm_boardconf.h - /rfnm/board.conf parser + TX path policy. CANONICAL COPY for the
// fleet's TX-capable daemons (TX_ALLOWED=embed locks TX to the embedded antenna; some boards
// are configured RX-only because their SMA feeds receive-side hardware). vsg/daemon/boardconf.h forwards here.
// Shell-style KEY=VALUE, keys BOARD and TX_ALLOWED. TX_ALLOWED values:
//   "0" / absent / anything unrecognized -> VSG_TXPOL_NONE  (no TX at all, fail-safe)
//   "embed"                              -> VSG_TXPOL_EMBED (TX on the embedded antenna only)
//   "1"                                  -> VSG_TXPOL_ALL   (no restriction)
// Plain C, header-only.
#ifndef VSG_BOARDCONF_H
#define VSG_BOARDCONF_H

#include <stdio.h>
#include <string.h>

enum vsg_tx_policy {
	VSG_TXPOL_NONE = 0,
	VSG_TXPOL_EMBED,
	VSG_TXPOL_ALL
};

static __attribute__((unused)) const char *vsg_tx_policy_str(enum vsg_tx_policy p) {
	return p == VSG_TXPOL_ALL ? "all" : (p == VSG_TXPOL_EMBED ? "embed" : "none");
}

static __attribute__((unused)) enum vsg_tx_policy vsg_tx_policy_parse(const char *v) {
	if(v && !strcmp(v, "1")) {
		return VSG_TXPOL_ALL;
	}
	if(v && !strcmp(v, "embed")) {
		return VSG_TXPOL_EMBED;
	}
	return VSG_TXPOL_NONE;
}

// parses path; fills board (BOARD=, "unknown" when absent) and returns the TX policy
// (TX_ALLOWED=). A missing/unreadable file is the fail-safe default: "unknown" + VSG_TXPOL_NONE.
static __attribute__((unused)) enum vsg_tx_policy vsg_boardconf_parse(const char *path, char *board, size_t board_len) {
	enum vsg_tx_policy pol = VSG_TXPOL_NONE;
	if(board && board_len) {
		snprintf(board, board_len, "unknown");
	}
	FILE *f = fopen(path, "r");
	if(!f) {
		return pol;
	}
	char line[256];
	while(fgets(line, sizeof(line), f)) {
		char *k = line;
		while(*k == ' ' || *k == '\t') {
			k++;
		}
		if(*k == '#' || *k == '\n' || *k == '\r' || !*k) {
			continue;
		}
		char *eq = strchr(k, '=');
		if(!eq) {
			continue;
		}
		*eq = 0;
		char *ke = eq;
		while(ke > k && (ke[-1] == ' ' || ke[-1] == '\t')) {
			*--ke = 0;
		}
		char *v = eq + 1;
		while(*v == ' ' || *v == '\t') {
			v++;
		}
		char *ve = v + strlen(v);
		while(ve > v && (ve[-1] == '\n' || ve[-1] == '\r' || ve[-1] == ' ' || ve[-1] == '\t')) {
			*--ve = 0;
		}
		if(ve - v >= 2 && ((*v == '"' && ve[-1] == '"') || (*v == '\'' && ve[-1] == '\''))) {
			*--ve = 0;
			v++;
		}
		if(!strcmp(k, "BOARD") && board && board_len) {
			snprintf(board, board_len, "%s", v);
		} else if(!strcmp(k, "TX_ALLOWED")) {
			pol = vsg_tx_policy_parse(v);
		}
	}
	fclose(f);
	return pol;
}

#endif
