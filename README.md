# tum_lanelet2_osm_fusion

[[_TOC_]]

## Overview

![image](doc/img/conflation_tool.png)

## Contact person

[Maximilian Leitenstern](mailto:maxi.leitenstern@tum.de)

## Configuration

Parameters used inside the package can be adjusted in `/config/lanelet2_osm.param.yaml`.\
Explanations on the parameters can be found in the comments and the documentation of the single [modules](#content).

## Visualization

Several ROS topics are published that can be visualized in RVIZ. The following table gives an overview of the topic and its description.
| Topic | Description |
| ----------- | ----------- |
| `/lof/map/osm_map_markers` | Street network downloaded from [OpenStreetMap](openstreetmap.org/). |
| `/lof/map/ll_map_markers` | Lanelet2-map enriched with attributes from [OpenStreetMap](openstreetmap.org/); Lanelets are colorized based on agreement between adjacent lanelets and lanes-tag of [OpenStreetMap](openstreetmap.org/). |
| `/lof/map/ll_map_new_markers` | Lanelet2-map enriched with attributes from [OpenStreetMap](openstreetmap.org/); Lanelets that are likely to be wrong were removed. |
| `/lof/traj/traj_master_markers` | Master trajectory -> to be defined in config-file (either GNSS- or SLAM trajectory) |
| `/lof/traj/traj_target_markers` | original target rajectory -> depending on selected master trajectory (either GNSS- or SLAM trajectory) |
| `/lof/traj/traj_align_markers` | target trajectory aligned to master with [Umeyama](https://web.stanford.edu/class/cs273/refs/umeyama.pdf) transformation or [PCL ICP](https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html) |
| `/lof/traj/traj_rs_markers` | target trajectory after [rubber-sheet](https://www.tandfonline.com/doi/abs/10.1559/152304085783915135)-transformation |
| `/lof/rs/geom_markers` | geometric information from rubber-sheeting (control points and constructed triangles) |
| `/clicked_point` | last 2 selected points by user to indicate chosen control point |
| `/lof/confl/geom_markers` | geometric information regarding conflation process (collapsed lanelet-map, buffers, matches) |
| `/lof/rs/pcd_map` | transformed point cloud map (only when using `kiss_icp_georef`) |

## How to use the package

1. Installation Development state:

   - clone the repository
   - pull development image

   ```shell
       docker pull gitlab.lrz.de:5005/sensor-fusion-slam/scanmatcher/lanelet2_osm_fusion_img
   ```

   - run the docker image
   ```shell
       ./run_docker.sh
   ```

   - enter the container 
   ```shell
       docker exec -it fervent_archimedes bash 
   ```

2. Necessary input parameters:
   - `traj_path` => path to GPS trajectory of the vehicle (format: txt-file with lat, lon)
   - `poses_path` => path to SLAM trajectory of the vehicle (KITTI-format, trajectories don't have to be synchronized over time)
   - `pcd_path` => path to pcd map corresponding to poses trajectory (only when using node `kiss_icp_georef`)
   - `map_path` => path to lanelet2 map corresponding to trajectories (map can have missing elements/attributes, only when using node `lanelet2_osm`)
   - `out_path` => path to saved the modified lanelet map (DEFAULT: /lanelet2_map.osm, only when using node `lanelet2_osm`)
3. Start the package

   - it is recommended to directly use the provided ROS launch file that starts the package itself and the visualization in RVIZ:

   ```shell
       ros2 launch tum_lanelet2_osm_fusion lanelet2_osm.launch.py traj_path:=<path-to-GPS-trajectory> poses_path:=<path-to-SLAM-trajectory>  map_path:=<path-to-lanelet-map> out_path:=<path-to-save-output-map>
   ```

   ```shell
       ros2 launch tum_lanelet2_osm_fusion kiss_icp_georef.launch.py traj_path:=<path-to-GPS-trajectory> poses_path:=<path-to-SLAM-trajectory>  pcd_path:=<path-to-pcd-map> pcd_out_path:=<path-to-save-pcd-map>
   ```

   - the launch file directly links to the parameter file in `/config/`.
   - If you do not want to use the launch file, you can start the package by running:

4. Select control points
   - after the trajectories are loaded and the target trajectory is roughly aligned to the master trajectory you are asked in the command window to select control points for the rubber-sheet transformation (the amount of points can be configured).
   - select the desired points using the `Publish Point` button in RVIZ and follow the instructions in the console.
5. Inspect results
   - results of the rubber-sheet transformation & lanelet map are visualized.
   - Inspect results and modify parameters if desired.
6. Manually finalize lanelet map (only when using node `lanelet2_osm`)
   - open a manual editor for lanelet2 maps (e.g. [VectorMapBuilder](https://tools.tier4.jp/feature/vector_map_builder_ll2/)) in parallel to RVIZ
   - import the exported map from `out_path`
   - close gaps in lanelet map and correct other mistakes based on visualization of map agreement with [OpenStreetMap](openstreetmap.org/) in RVIZ

## Test Data

The test data in `/test` is from the EDGAR research vehicle (GPS trajectory). The SLAM poses are generated by [KISS-ICP](https://github.com/PRBonn/kiss-icp) in combination with [interactive SLAM](https://github.com/SMRT-AIST/interactive_slam). The lanelet2-map is created manually with [VectorMapBuilder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) by TieriV.

## Content

Detailed documentation of the modules can be found below.

1. [Geometric Alignment](doc/alignment.md)

2. [Preprocessing](doc/preprocessing.md)

3. [Matching](doc/matching.md)

4. [Conflation](doc/conflation.md)

5. [Analysis](doc/analysis.md)
