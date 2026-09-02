#pragma once
// map_store.h -- on-disk map layout (geometry/ + overlays/ + snapshots/) and the
// operations over it: list, atomic geometry commit, snapshot create/restore/list/prune,
// and a generic file-history helper. ROS-free; all filesystem access is injected via
// MapFs so the core is unit-testable against an in-memory tree. Replaces VersionStore.
#include <functional>
#include <string>
#include <vector>

#include "g1_map_manager/core/map_lineage.h"

namespace g1_map_manager {

struct MapFs {
  std::function<std::vector<std::string>(const std::string&)> listDirs;
  std::function<std::vector<std::string>(const std::string&)> listFiles;
  std::function<std::string(const std::string&)> read;
  std::function<void(const std::string&, const std::string&)> writeAtomic;
  std::function<bool(const std::string&)> exists;
  std::function<void(const std::string&, const std::string&)> copyTree;
  std::function<void(const std::string&, const std::string&)> moveTree;
  std::function<void(const std::string&)> removeTree;
};

class MapStore {
 public:
  MapStore(std::string maps_root, MapFs fs);

  // Pure path composition.
  std::string mapDir(const std::string& name) const;
  std::string geometryDir(const std::string& name) const;
  std::string geometryTmpDir(const std::string& name) const;
  std::string overlaysDir(const std::string& name) const;
  std::string historyDir(const std::string& name) const;
  std::string snapshotsDir(const std::string& name) const;
  std::string snapshotDir(const std::string& name, const std::string& id) const;

  std::vector<std::string> listMaps() const;
  MapLineage readLineage(const std::string& name) const;

  // Atomic geometry swap: caller has written a full map into geometryTmpDir(name).
  void commitGeometry(const std::string& name) const;

  // Snapshots (full geometry/ + overlays/ copies).
  void createSnapshot(const std::string& name, const std::string& id) const;
  void restoreSnapshot(const std::string& name, const std::string& id) const;
  std::vector<std::string> listSnapshots(const std::string& name) const;  // sorted
  void pruneSnapshots(const std::string& name, const std::string& suffix, int keep) const;
  void removeSnapshot(const std::string& name, const std::string& id) const;

 private:
  std::string maps_root_;
  MapFs fs_;
};

// Generic history-on-write: back up the current file (if any) to
// <history_dir>/<base>.<timestamp><ext>, write `data` to `path` atomically, then prune
// <history_dir> entries whose name starts with "<base>." to the newest `keep`.
// `timestamp` is supplied by the caller (no time in the core). keep <= 0 disables history.
void writeFileWithHistory(const MapFs& fs, const std::string& path,
                          const std::string& history_dir, const std::string& timestamp,
                          const std::string& data, int keep);

}  // namespace g1_map_manager
