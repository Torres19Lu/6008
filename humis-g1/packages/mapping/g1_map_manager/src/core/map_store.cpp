// map_store.cpp -- geometry/overlays/snapshots layout: paths, listMaps, atomic geometry
// commit, snapshot create/restore/list/prune, and a generic file-history helper.

#include "g1_map_manager/core/map_store.h"

#include <algorithm>
#include <utility>

namespace g1_map_manager {

namespace {
std::string baseName(const std::string& path) {
  const auto s = path.find_last_of('/');
  return s == std::string::npos ? path : path.substr(s + 1);
}
}  // namespace

MapStore::MapStore(std::string maps_root, MapFs fs)
    : maps_root_(std::move(maps_root)), fs_(std::move(fs)) {}

std::string MapStore::mapDir(const std::string& name) const {
  return maps_root_ + "/" + name;
}
std::string MapStore::geometryDir(const std::string& name) const {
  return mapDir(name) + "/geometry";
}
std::string MapStore::geometryTmpDir(const std::string& name) const {
  return mapDir(name) + "/geometry.tmp";
}
std::string MapStore::overlaysDir(const std::string& name) const {
  return mapDir(name) + "/overlays";
}
std::string MapStore::historyDir(const std::string& name) const {
  return overlaysDir(name) + "/.history";
}
std::string MapStore::snapshotsDir(const std::string& name) const {
  return mapDir(name) + "/snapshots";
}
std::string MapStore::snapshotDir(const std::string& name, const std::string& id) const {
  return snapshotsDir(name) + "/" + id;
}

std::vector<std::string> MapStore::listMaps() const {
  std::vector<std::string> maps = fs_.listDirs(maps_root_);
  std::sort(maps.begin(), maps.end());
  return maps;
}

MapLineage MapStore::readLineage(const std::string& name) const {
  return parseLineage(fs_.read(geometryDir(name) + "/lineage.yaml"));
}

void MapStore::commitGeometry(const std::string& name) const {
  const std::string live = geometryDir(name);
  const std::string tmp = geometryTmpDir(name);
  const std::string old = mapDir(name) + "/geometry.old";
  if (fs_.exists(old)) fs_.removeTree(old);   // clean a prior interrupted commit
  if (fs_.exists(live)) fs_.moveTree(live, old);
  fs_.moveTree(tmp, live);
  if (fs_.exists(old)) fs_.removeTree(old);
}

void MapStore::createSnapshot(const std::string& name, const std::string& id) const {
  const std::string tmp = snapshotDir(name, id) + ".tmp";
  const std::string dst = snapshotDir(name, id);
  if (fs_.exists(tmp)) fs_.removeTree(tmp);
  fs_.copyTree(geometryDir(name), tmp + "/geometry");
  if (fs_.exists(overlaysDir(name))) {
    fs_.copyTree(overlaysDir(name), tmp + "/overlays");
    fs_.removeTree(tmp + "/overlays/.history");  // history is not part of a snapshot
  }
  if (fs_.exists(dst)) fs_.removeTree(dst);
  fs_.moveTree(tmp, dst);
}

void MapStore::restoreSnapshot(const std::string& name, const std::string& id) const {
  const std::string snap = snapshotDir(name, id);
  if (fs_.exists(geometryTmpDir(name))) fs_.removeTree(geometryTmpDir(name));
  fs_.copyTree(snap + "/geometry", geometryTmpDir(name));
  commitGeometry(name);
  if (fs_.exists(snap + "/overlays")) {
    if (fs_.exists(overlaysDir(name))) fs_.removeTree(overlaysDir(name));
    fs_.copyTree(snap + "/overlays", overlaysDir(name));
  }
}

std::vector<std::string> MapStore::listSnapshots(const std::string& name) const {
  std::vector<std::string> ids = fs_.listDirs(snapshotsDir(name));
  std::sort(ids.begin(), ids.end());
  return ids;
}

void MapStore::pruneSnapshots(const std::string& name, const std::string& suffix,
                              int keep) const {
  std::vector<std::string> matching;
  for (const std::string& id : listSnapshots(name)) {
    if (id.size() >= suffix.size() &&
        id.compare(id.size() - suffix.size(), suffix.size(), suffix) == 0) {
      matching.push_back(id);
    }
  }
  const int drop = static_cast<int>(matching.size()) - keep;
  for (int i = 0; i < drop; ++i) fs_.removeTree(snapshotDir(name, matching[i]));
}

void MapStore::removeSnapshot(const std::string& name, const std::string& id) const {
  fs_.removeTree(snapshotDir(name, id));
}

void writeFileWithHistory(const MapFs& fs, const std::string& path,
                          const std::string& history_dir, const std::string& timestamp,
                          const std::string& data, int keep) {
  const std::string current = fs.read(path);
  if (keep > 0 && !current.empty()) {
    const std::string fname = baseName(path);          // e.g. nav_points.yaml
    const auto dot = fname.find_last_of('.');
    const std::string base = dot == std::string::npos ? fname : fname.substr(0, dot);
    const std::string ext = dot == std::string::npos ? "" : fname.substr(dot);  // ".yaml"
    fs.writeAtomic(history_dir + "/" + base + "." + timestamp + ext, current);

    std::vector<std::string> mine;
    for (const std::string& e : fs.listFiles(history_dir)) {
      if (e.rfind(base + ".", 0) == 0) mine.push_back(e);
    }
    std::sort(mine.begin(), mine.end());               // oldest first by timestamp
    const int drop = static_cast<int>(mine.size()) - keep;
    for (int idx = 0; idx < drop; ++idx) fs.removeTree(history_dir + "/" + mine[idx]);
  }
  fs.writeAtomic(path, data);
}

}  // namespace g1_map_manager
