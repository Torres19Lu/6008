// MapStore tests with an in-memory tree (flat path->content; dirs implied by prefixes).

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "g1_map_manager/core/map_store.h"

using namespace g1_map_manager;

namespace {

struct FakeFs {
  std::map<std::string, std::string> files;  // absolute path -> content

  MapFs iface() {
    MapFs fs;
    fs.listDirs = [this](const std::string& dir) {
      std::set<std::string> out;
      const std::string pre = dir + "/";
      for (const auto& kv : files) {
        if (kv.first.rfind(pre, 0) != 0) continue;
        const std::string rest = kv.first.substr(pre.size());
        const auto slash = rest.find('/');
        if (slash != std::string::npos) out.insert(rest.substr(0, slash));
      }
      return std::vector<std::string>(out.begin(), out.end());
    };
    fs.listFiles = [this](const std::string& dir) {
      std::vector<std::string> out;
      const std::string pre = dir + "/";
      for (const auto& kv : files) {
        if (kv.first.rfind(pre, 0) != 0) continue;
        const std::string rest = kv.first.substr(pre.size());
        if (rest.find('/') == std::string::npos) out.push_back(rest);
      }
      std::sort(out.begin(), out.end());
      return out;
    };
    fs.read = [this](const std::string& path) {
      auto it = files.find(path);
      return it == files.end() ? std::string{} : it->second;
    };
    fs.writeAtomic = [this](const std::string& path, const std::string& data) {
      files[path] = data;
    };
    fs.exists = [this](const std::string& path) {
      if (files.count(path)) return true;
      const std::string pre = path + "/";
      for (const auto& kv : files)
        if (kv.first.rfind(pre, 0) == 0) return true;
      return false;
    };
    fs.copyTree = [this](const std::string& src, const std::string& dst) {
      const std::string pre = src + "/";
      std::map<std::string, std::string> add;
      for (const auto& kv : files)
        if (kv.first.rfind(pre, 0) == 0)
          add[dst + "/" + kv.first.substr(pre.size())] = kv.second;
      for (auto& kv : add) files[kv.first] = kv.second;
    };
    fs.removeTree = [this](const std::string& path) {
      const std::string pre = path + "/";
      for (auto it = files.begin(); it != files.end();)
        it = (it->first == path || it->first.rfind(pre, 0) == 0) ? files.erase(it) : ++it;
    };
    // Operate on `files` directly so the lambda does not capture the local MapFs.
    fs.moveTree = [this](const std::string& src, const std::string& dst) {
      const std::string pre = src + "/";
      std::map<std::string, std::string> moved;
      for (auto it = files.begin(); it != files.end();) {
        if (it->first == src || it->first.rfind(pre, 0) == 0) {
          const std::string rel =
              it->first.size() > src.size() ? it->first.substr(src.size()) : "";
          moved[dst + rel] = it->second;
          it = files.erase(it);
        } else {
          ++it;
        }
      }
      for (auto& kv : moved) files[kv.first] = kv.second;
    };
    return fs;
  }
};

MapStore makeStore(FakeFs* fs) { return MapStore("/maps", fs->iface()); }

}  // namespace

TEST(MapStore, ComposesPaths) {
  FakeFs fs;
  const MapStore s = makeStore(&fs);
  EXPECT_EQ(s.geometryDir("eee-b4"), "/maps/eee-b4/geometry");
  EXPECT_EQ(s.geometryTmpDir("eee-b4"), "/maps/eee-b4/geometry.tmp");
  EXPECT_EQ(s.overlaysDir("eee-b4"), "/maps/eee-b4/overlays");
  EXPECT_EQ(s.historyDir("eee-b4"), "/maps/eee-b4/overlays/.history");
  EXPECT_EQ(s.snapshotDir("eee-b4", "20260624-093000-x"),
            "/maps/eee-b4/snapshots/20260624-093000-x");
}

TEST(MapStore, ListsMaps) {
  FakeFs fs;
  fs.files["/maps/floor1/geometry/manifest.yaml"] = "name: floor1\n";
  fs.files["/maps/floor2/geometry/manifest.yaml"] = "name: floor2\n";
  const MapStore s = makeStore(&fs);
  EXPECT_EQ(s.listMaps(), (std::vector<std::string>{"floor1", "floor2"}));
}

TEST(MapStore, ReadsLineage) {
  FakeFs fs;
  fs.files["/maps/floor1/geometry/lineage.yaml"] =
      "name: floor1\nreason: initial\nparent: \"\"\n";
  const MapStore s = makeStore(&fs);
  EXPECT_EQ(s.readLineage("floor1").name, "floor1");
  EXPECT_EQ(s.readLineage("floor1").reason, "initial");
}

TEST(MapStore, CommitGeometrySwapsTmpIntoPlace) {
  FakeFs fs;
  fs.files["/maps/m/geometry/manifest.yaml"] = "name: m\nnum_keyframes: 1\n";
  fs.files["/maps/m/geometry/keyframes/000000.pcd"] = "OLD";
  fs.files["/maps/m/geometry.tmp/manifest.yaml"] = "name: m\nnum_keyframes: 2\n";
  fs.files["/maps/m/geometry.tmp/keyframes/000000.pcd"] = "NEW0";
  fs.files["/maps/m/geometry.tmp/keyframes/000001.pcd"] = "NEW1";
  const MapStore s = makeStore(&fs);

  s.commitGeometry("m");

  EXPECT_EQ(fs.files["/maps/m/geometry/manifest.yaml"], "name: m\nnum_keyframes: 2\n");
  EXPECT_EQ(fs.files["/maps/m/geometry/keyframes/000000.pcd"], "NEW0");
  EXPECT_EQ(fs.files["/maps/m/geometry/keyframes/000001.pcd"], "NEW1");
  EXPECT_FALSE(fs.iface().exists("/maps/m/geometry.tmp"));
  EXPECT_FALSE(fs.iface().exists("/maps/m/geometry.old"));
}

TEST(MapStore, CommitGeometryFirstTimeNoPriorGeometry) {
  FakeFs fs;
  fs.files["/maps/m/geometry.tmp/manifest.yaml"] = "name: m\n";
  const MapStore s = makeStore(&fs);
  s.commitGeometry("m");
  EXPECT_EQ(fs.files["/maps/m/geometry/manifest.yaml"], "name: m\n");
  EXPECT_FALSE(fs.iface().exists("/maps/m/geometry.tmp"));
}

TEST(MapStore, CreateSnapshotCopiesGeometryAndOverlays) {
  FakeFs fs;
  fs.files["/maps/m/geometry/manifest.yaml"] = "name: m\n";
  fs.files["/maps/m/geometry/keyframes/000000.pcd"] = "G0";
  fs.files["/maps/m/overlays/nav_points.yaml"] = "NP";
  fs.files["/maps/m/overlays/.history/nav_points.1.yaml"] = "OLD";  // must NOT be copied
  const MapStore s = makeStore(&fs);

  s.createSnapshot("m", "20260624-093000-pre-incremental");
  const std::string snap = "/maps/m/snapshots/20260624-093000-pre-incremental";
  EXPECT_EQ(fs.files[snap + "/geometry/manifest.yaml"], "name: m\n");
  EXPECT_EQ(fs.files[snap + "/geometry/keyframes/000000.pcd"], "G0");
  EXPECT_EQ(fs.files[snap + "/overlays/nav_points.yaml"], "NP");
  EXPECT_FALSE(fs.iface().exists(snap + "/overlays/.history"));
  EXPECT_FALSE(fs.iface().exists("/maps/m/snapshots/20260624-093000-pre-incremental.tmp"));
}

TEST(MapStore, RestoreSnapshotReplacesLive) {
  FakeFs fs;
  fs.files["/maps/m/geometry/manifest.yaml"] = "NEWBAD";
  fs.files["/maps/m/overlays/nav_points.yaml"] = "NEWBAD";
  fs.files["/maps/m/snapshots/s1/geometry/manifest.yaml"] = "GOOD";
  fs.files["/maps/m/snapshots/s1/overlays/nav_points.yaml"] = "GOODNP";
  const MapStore s = makeStore(&fs);

  s.restoreSnapshot("m", "s1");
  EXPECT_EQ(fs.files["/maps/m/geometry/manifest.yaml"], "GOOD");
  EXPECT_EQ(fs.files["/maps/m/overlays/nav_points.yaml"], "GOODNP");
  EXPECT_FALSE(fs.iface().exists("/maps/m/geometry.tmp"));
}

TEST(MapStore, ListSnapshotsSorted) {
  FakeFs fs;
  fs.files["/maps/m/snapshots/20260620-140500-initial/geometry/manifest.yaml"] = "a";
  fs.files["/maps/m/snapshots/20260624-093000-pre-incremental/geometry/manifest.yaml"] = "b";
  const MapStore s = makeStore(&fs);
  EXPECT_EQ(s.listSnapshots("m"),
            (std::vector<std::string>{"20260620-140500-initial",
                                      "20260624-093000-pre-incremental"}));
}

TEST(MapStore, PruneSnapshotsKeepsNewestMatchingSuffix) {
  FakeFs fs;
  for (const char* id : {"20260101-000000-pre-incremental",
                         "20260102-000000-pre-incremental",
                         "20260103-000000-pre-incremental",
                         "20260104-000000-milestone"}) {
    fs.files[std::string("/maps/m/snapshots/") + id + "/geometry/manifest.yaml"] = "x";
  }
  const MapStore s = makeStore(&fs);
  s.pruneSnapshots("m", "-pre-incremental", 1);  // keep only the newest auto snapshot

  EXPECT_FALSE(fs.iface().exists("/maps/m/snapshots/20260101-000000-pre-incremental"));
  EXPECT_FALSE(fs.iface().exists("/maps/m/snapshots/20260102-000000-pre-incremental"));
  EXPECT_TRUE(fs.iface().exists("/maps/m/snapshots/20260103-000000-pre-incremental"));
  EXPECT_TRUE(fs.iface().exists("/maps/m/snapshots/20260104-000000-milestone"));  // untouched
}

TEST(MapStore, WriteFileWithHistoryBacksUpThenWrites) {
  FakeFs fs;
  fs.files["/maps/m/overlays/nav_points.yaml"] = "V1";
  const MapFs i = fs.iface();
  writeFileWithHistory(i, "/maps/m/overlays/nav_points.yaml",
                       "/maps/m/overlays/.history", "20260624-093000", "V2", 5);
  EXPECT_EQ(fs.files["/maps/m/overlays/nav_points.yaml"], "V2");
  EXPECT_EQ(fs.files["/maps/m/overlays/.history/nav_points.20260624-093000.yaml"], "V1");
}

TEST(MapStore, WriteFileWithHistoryNoBackupWhenAbsent) {
  FakeFs fs;
  const MapFs i = fs.iface();
  writeFileWithHistory(i, "/maps/m/overlays/nav_points.yaml",
                       "/maps/m/overlays/.history", "20260624-093000", "V1", 5);
  EXPECT_EQ(fs.files["/maps/m/overlays/nav_points.yaml"], "V1");
  EXPECT_FALSE(i.exists("/maps/m/overlays/.history"));
}

TEST(MapStore, WriteFileWithHistoryPrunesToKeep) {
  FakeFs fs;
  fs.files["/maps/m/overlays/.history/nav_points.20260101-000000.yaml"] = "h1";
  fs.files["/maps/m/overlays/.history/nav_points.20260102-000000.yaml"] = "h2";
  fs.files["/maps/m/overlays/.history/metadata.20260101-000000.yaml"] = "other";  // untouched
  fs.files["/maps/m/overlays/nav_points.yaml"] = "cur";
  const MapFs i = fs.iface();
  writeFileWithHistory(i, "/maps/m/overlays/nav_points.yaml",
                       "/maps/m/overlays/.history", "20260103-000000", "new", 2);
  EXPECT_FALSE(i.exists("/maps/m/overlays/.history/nav_points.20260101-000000.yaml"));
  EXPECT_TRUE(i.exists("/maps/m/overlays/.history/nav_points.20260102-000000.yaml"));
  EXPECT_TRUE(i.exists("/maps/m/overlays/.history/nav_points.20260103-000000.yaml"));
  EXPECT_TRUE(i.exists("/maps/m/overlays/.history/metadata.20260101-000000.yaml"));
}

TEST(MapStore, RemoveSnapshotDeletesOne) {
  FakeFs fs;
  fs.files["/maps/m/snapshots/s1/geometry/manifest.yaml"] = "x";
  fs.files["/maps/m/snapshots/s2/geometry/manifest.yaml"] = "y";
  const MapStore s = makeStore(&fs);
  s.removeSnapshot("m", "s1");
  EXPECT_FALSE(fs.iface().exists("/maps/m/snapshots/s1"));
  EXPECT_TRUE(fs.iface().exists("/maps/m/snapshots/s2"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
