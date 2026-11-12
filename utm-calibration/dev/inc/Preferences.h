#pragma once
#include <map>
#include <string>
#include <vector>
#include <cstdio>
// file-backed so settings survive a simulated reset
class Preferences {
  std::string path;
  std::map<std::string, std::vector<uint8_t>> kv;
  void save() { FILE* f = fopen(path.c_str(), "wb"); if (!f) return; for (auto& p : kv) { uint32_t a = p.first.size(), b = p.second.size(); fwrite(&a, 4, 1, f); fwrite(p.first.data(), 1, a, f); fwrite(&b, 4, 1, f); fwrite(p.second.data(), 1, b, f); } fclose(f); }
 public:
  bool begin(const char* ns, bool) { path = std::string("prefs_") + ns + ".bin"; FILE* f = fopen(path.c_str(), "rb"); if (!f) return true; uint32_t a, b; while (fread(&a, 4, 1, f) == 1) { std::string k(a, 0); fread(&k[0], 1, a, f); fread(&b, 4, 1, f); std::vector<uint8_t> v(b); fread(v.data(), 1, b, f); kv[k] = v; } fclose(f); return true; }
  size_t getBytesLength(const char* k) { auto it = kv.find(k); return it == kv.end() ? 0 : it->second.size(); }
  size_t getBytes(const char* k, void* p, size_t n) { auto it = kv.find(k); if (it == kv.end()) return 0; size_t m = std::min(n, it->second.size()); memcpy(p, it->second.data(), m); return m; }
  size_t putBytes(const char* k, const void* p, size_t n) { kv[k] = std::vector<uint8_t>((const uint8_t*)p, (const uint8_t*)p + n); save(); return n; }
};
