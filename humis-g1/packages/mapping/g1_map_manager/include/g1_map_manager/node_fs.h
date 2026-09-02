#pragma once
// node_fs.h -- node-only (NOT part of the ROS-free core): build a MapFs backed by
// std::filesystem for the map_manager and map_edit nodes. Header-only + inline so both
// nodes share one implementation without a duplicate symbol. No ROS dependency; pure
// std::filesystem + fstream.
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "g1_map_manager/core/map_store.h"

namespace g1_map_manager {

inline MapFs makeMapFs() {
  namespace fs = std::filesystem;
  MapFs f;
  f.listDirs = [](const std::string& dir) {
    std::vector<std::string> out;
    std::error_code ec;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
      if (it->is_directory(ec)) out.push_back(it->path().filename().string());
    }
    std::sort(out.begin(), out.end());
    return out;
  };
  f.listFiles = [](const std::string& dir) {
    std::vector<std::string> out;
    std::error_code ec;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
      if (it->is_regular_file(ec)) out.push_back(it->path().filename().string());
    }
    std::sort(out.begin(), out.end());
    return out;
  };
  f.read = [](const std::string& path) {
    std::ifstream is(path, std::ios::binary);
    if (!is) return std::string{};
    std::ostringstream ss;
    ss << is.rdbuf();
    return ss.str();
  };
  f.writeAtomic = [](const std::string& path, const std::string& data) {
    namespace fs = std::filesystem;
    const fs::path target(path);
    std::error_code ec;
    if (target.has_parent_path()) fs::create_directories(target.parent_path(), ec);
    const fs::path tmp = target.string() + ".tmp";
    {
      std::ofstream os(tmp, std::ios::binary | std::ios::trunc);
      os << data;
    }
    fs::rename(tmp, target, ec);
    if (ec) fs::copy_file(tmp, target, fs::copy_options::overwrite_existing, ec);
  };
  f.exists = [](const std::string& p) {
    std::error_code ec;
    return std::filesystem::exists(p, ec);
  };
  f.copyTree = [](const std::string& src, const std::string& dst) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(fs::path(dst).parent_path(), ec);
    fs::copy(src, dst,
             fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
  };
  f.moveTree = [](const std::string& src, const std::string& dst) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(fs::path(dst).parent_path(), ec);
    fs::rename(src, dst, ec);
    if (ec) {  // cross-device or other: fall back to copy + remove
      ec.clear();
      fs::copy(src, dst,
               fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
      fs::remove_all(src, ec);
    }
  };
  f.removeTree = [](const std::string& p) {
    std::error_code ec;
    std::filesystem::remove_all(p, ec);
  };
  return f;
}

}  // namespace g1_map_manager
