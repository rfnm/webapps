// rfnm_json.h - tiny JSON parse + emit for the fleet daemons. CANONICAL COPY; origin
// webgui/daemon/appd.cpp (appd migrates onto this header in a later wave - until then keep
// the two in sync when touching parser semantics). Header-only, namespace rfnm, no deps.
// Parser: objects/arrays/str/num/bool/null, depth-limited (16), BMP \u escapes -> UTF-8.
// Emit: jesc/jkv/jkv_raw/jarr - the flat snprintf-free JSON building style the fleet uses.
#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace rfnm {

struct jv {
	enum { NUL, BOO, NUM, STR, OBJ, ARR } t = NUL;
	bool b = false;
	double n = 0;
	std::string s;
	std::vector<std::pair<std::string, jv>> o;
	std::vector<jv> a;
	const jv *get(const std::string &k) const {
		if(t != OBJ) {
			return nullptr;
		}
		for(const auto &kv : o) {
			if(kv.first == k) {
				return &kv.second;
			}
		}
		return nullptr;
	}
	std::string str(const std::string &k, const std::string &def = "") const {
		const jv *v = get(k);
		return v && v->t == STR ? v->s : def;
	}
	double num(const std::string &k, double def = 0) const {
		const jv *v = get(k);
		return v && v->t == NUM ? v->n : def;
	}
	bool boolean(const std::string &k, bool def = false) const {
		const jv *v = get(k);
		return v && v->t == BOO ? v->b : def;
	}
};

inline void jskip(const std::string &s, size_t &p) {
	while(p < s.size() && (s[p] == ' ' || s[p] == '\t' || s[p] == '\n' || s[p] == '\r')) {
		p++;
	}
}

inline bool jparse_str(const std::string &s, size_t &p, std::string &out) {
	if(p >= s.size() || s[p] != '"') {
		return false;
	}
	p++;
	out.clear();
	while(p < s.size()) {
		char c = s[p];
		if(c == '"') {
			p++;
			return true;
		}
		if(c == '\\') {
			if(p + 1 >= s.size()) {
				return false;
			}
			char e = s[p + 1];
			p += 2;
			switch(e) {
				case '"': out += '"'; break;
				case '\\': out += '\\'; break;
				case '/': out += '/'; break;
				case 'n': out += '\n'; break;
				case 't': out += '\t'; break;
				case 'r': out += '\r'; break;
				case 'b': out += '\b'; break;
				case 'f': out += '\f'; break;
				case 'u': {   // decode BMP escapes to UTF-8; good enough for config documents
					if(p + 4 > s.size()) {
						return false;
					}
					unsigned v = (unsigned)strtoul(s.substr(p, 4).c_str(), nullptr, 16);
					p += 4;
					if(v < 0x80) {
						out += (char)v;
					} else if(v < 0x800) {
						out += (char)(0xC0 | (v >> 6));
						out += (char)(0x80 | (v & 0x3F));
					} else {
						out += (char)(0xE0 | (v >> 12));
						out += (char)(0x80 | ((v >> 6) & 0x3F));
						out += (char)(0x80 | (v & 0x3F));
					}
					break;
				}
				default: return false;
			}
		} else {
			out += c;
			p++;
		}
	}
	return false;
}

inline bool jparse_val(const std::string &s, size_t &p, jv &out, int depth) {
	if(depth > 16) {
		return false;
	}
	jskip(s, p);
	if(p >= s.size()) {
		return false;
	}
	char c = s[p];
	if(c == '"') {
		out.t = jv::STR;
		return jparse_str(s, p, out.s);
	}
	if(c == '{') {
		out.t = jv::OBJ;
		p++;
		jskip(s, p);
		if(p < s.size() && s[p] == '}') {
			p++;
			return true;
		}
		while(p < s.size()) {
			jskip(s, p);
			std::string key;
			if(!jparse_str(s, p, key)) {
				return false;
			}
			jskip(s, p);
			if(p >= s.size() || s[p] != ':') {
				return false;
			}
			p++;
			jv val;
			if(!jparse_val(s, p, val, depth + 1)) {
				return false;
			}
			out.o.emplace_back(std::move(key), std::move(val));
			jskip(s, p);
			if(p < s.size() && s[p] == ',') {
				p++;
				continue;
			}
			if(p < s.size() && s[p] == '}') {
				p++;
				return true;
			}
			return false;
		}
		return false;
	}
	if(c == '[') {
		out.t = jv::ARR;
		p++;
		jskip(s, p);
		if(p < s.size() && s[p] == ']') {
			p++;
			return true;
		}
		while(p < s.size()) {
			jv val;
			if(!jparse_val(s, p, val, depth + 1)) {
				return false;
			}
			out.a.push_back(std::move(val));
			jskip(s, p);
			if(p < s.size() && s[p] == ',') {
				p++;
				continue;
			}
			if(p < s.size() && s[p] == ']') {
				p++;
				return true;
			}
			return false;
		}
		return false;
	}
	if(c == 't' && s.compare(p, 4, "true") == 0) {
		out.t = jv::BOO;
		out.b = true;
		p += 4;
		return true;
	}
	if(c == 'f' && s.compare(p, 5, "false") == 0) {
		out.t = jv::BOO;
		out.b = false;
		p += 5;
		return true;
	}
	if(c == 'n' && s.compare(p, 4, "null") == 0) {
		out.t = jv::NUL;
		p += 4;
		return true;
	}
	char *end = nullptr;
	double v = strtod(s.c_str() + p, &end);
	if(end == s.c_str() + p) {
		return false;
	}
	out.t = jv::NUM;
	out.n = v;
	p = (size_t)(end - s.c_str());
	return true;
}

inline bool jparse(const std::string &s, jv &out) {
	size_t p = 0;
	if(!jparse_val(s, p, out, 0)) {
		return false;
	}
	jskip(s, p);
	return p == s.size();
}

// ---- emit -------------------------------------------------------------------------------
inline std::string jesc(const std::string &s) {
	std::string o;
	o.reserve(s.size() + 8);
	size_t i = 0;
	while(i < s.size()) {
		unsigned char c = (unsigned char)s[i];
		if(c == '"' || c == '\\') {
			o += '\\';
			o += (char)c;
			i++;
		} else if(c < 0x20) {
			char b[8];
			snprintf(b, sizeof(b), "\\u%04x", c);
			o += b;
			i++;
		} else if(c < 0x80) {
			o += (char)c;
			i++;
		} else {
			int cont = (c & 0xE0) == 0xC0 ? 1 : (c & 0xF0) == 0xE0 ? 2 : (c & 0xF8) == 0xF0 ? 3 : -1;
			bool ok = cont > 0 && c != 0xC0 && c != 0xC1 && c <= 0xF4 && i + (size_t)cont < s.size();
			for(int k = 1; ok && k <= cont; k++) {
				ok = ((unsigned char)s[i + k] & 0xC0) == 0x80;
			}
			if(ok) {
				o.append(s, i, (size_t)cont + 1);
				i += (size_t)cont + 1;
			} else {
				o += '?';
				i++;
			}
		}
	}
	return o;
}

inline void jkv(std::string &j, const char *key, const std::string &val, bool &first) {
	j += first ? "" : ",";
	first = false;
	j += "\"";
	j += key;
	j += "\":\"" + jesc(val) + "\"";
}

inline void jkv_raw(std::string &j, const char *key, const std::string &raw, bool &first) {
	j += first ? "" : ",";
	first = false;
	j += "\"";
	j += key;
	j += "\":" + raw;
}

inline std::string jarr(const std::vector<std::string> &lines) {
	std::string j = "[";
	for(size_t i = 0; i < lines.size(); i++) {
		j += (i ? "," : "") + ("\"" + jesc(lines[i]) + "\"");
	}
	return j + "]";
}

} // namespace rfnm
