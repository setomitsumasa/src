// ERC — offline .pcd downsampler.
//
// FAST-LIO's saved scans.pcd is the raw, full-resolution accumulated cloud (tens of
// millions of points even for a small room) -- fine for GICP (map_anchor re-downsamples
// internally at its own voxel_leaf anyway) but too heavy for RViz to hold/render, which
// can OOM-kill the RViz process. Run this once when promoting a fresh scan to the
// canonical maps/prior_map.pcd; it drops per-point normals/curvature (unused downstream)
// and voxel-decimates to a size RViz can carry comfortably.
//
// Field use: edit config/downsample_map.yaml (input_pcd / output_pcd / leaf_size), then
// just run:   ros2 run ares_erc_bringup downsample_map
// Parameters live in the file so the terminal command stays short and repeatable outdoors.
// Optional: pass a path to use a different yaml config instead of the default one.

#include <cstdlib>
#include <iostream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

namespace
{
std::string expand_user(const std::string & path)
{
  if (path.size() >= 2 && path[0] == '~' && path[1] == '/') {
    const char * home = std::getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string config_path;
  if (argc >= 2) {
    config_path = argv[1];
  } else {
    config_path = ament_index_cpp::get_package_share_directory("ares_erc_bringup") +
      "/config/downsample_map.yaml";
  }

  YAML::Node cfg;
  try {
    cfg = YAML::LoadFile(config_path);
  } catch (const std::exception & ex) {
    std::cerr << "failed to read config " << config_path << ": " << ex.what() << "\n";
    std::cerr << "usage: downsample_map [config.yaml]  "
                 "(default: config/downsample_map.yaml in this package)\n";
    return 1;
  }
  if (!cfg["input_pcd"] || !cfg["output_pcd"]) {
    std::cerr << config_path << " must set input_pcd and output_pcd\n";
    return 1;
  }
  const std::string in_path = expand_user(cfg["input_pcd"].as<std::string>());
  const std::string out_path = expand_user(cfg["output_pcd"].as<std::string>());
  const float leaf = cfg["leaf_size"] ? cfg["leaf_size"].as<float>() : 0.05f;

  pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_path, *raw) < 0) {
    std::cerr << "failed to load " << in_path << "\n";
    return 1;
  }
  if (raw->empty()) {
    std::cerr << in_path << " loaded but has 0 points\n";
    return 1;
  }
  std::cout << "loaded " << raw->size() << " points from " << in_path << "\n";

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setLeafSize(leaf, leaf, leaf);
  vg.setInputCloud(raw);
  vg.filter(*filtered);

  pcl::io::savePCDFileBinary(out_path, *filtered);
  const double pct =
    100.0 * static_cast<double>(filtered->size()) / static_cast<double>(raw->size());
  std::cout << "wrote " << filtered->size() << " points (leaf=" << leaf << " m) to "
            << out_path << " (" << pct << "% of original)\n";
  return 0;
}
