# Lanelet2_OSM_Fusion
[![Linux](https://img.shields.io/badge/os-linux-blue.svg)](https://www.linux.org/)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
[![ROS2humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/index.html)

## Overview

![image](doc/img/conflation_tool.png)

<details>
<summary> <h2> 🐋 Docker Setup </h2> </summary>

### Package Design
This package is designed as a standalone ROS2 package. It was developed with ROS2 humble. For easier handling of dependencies, a docker environment is provided that sets up everything and builds the package.
As the package is designed for use in combination with [Autoware](https://github.com/autowarefoundation/autoware), the source code can also be build within the [Autoware](https://github.com/autowarefoundation/autoware) docker environment.

### Setup

1. Clone the repository by running
   ```bash
   git clone https://github.com/TUMFTM/Lanelet2_OSM_Fusion.git
   ```
2. Go to the rood directory of the repository
   ```bash
   cd Lanelet2_OSM_Fusion/
   ```
3. Build the docker image
   ```bash
   ./build_docker.sh
   ```
4. Run the container and mount your data by adjusting the corresponding line within ```run_docker.sh```
   ```bash
   ./run_docker.sh
   ```
</details>

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

<details> 
<summary> <h2> 🖥 How to Use the Package </h2> </summary>

* the package contains two executables with corresponding ROS2 launch file:
  * lanelet2_osm
    * provide all functionality described in the publication and the pipeline overview
    * additional possibility to georeference the point cloud map corresponding to the lanelet map, but without its visualization  
  * kiss_icp_georef
    * provides the possibility to georeference the SLAM poses and the corresponding point cloud map withouth the need of a lanelet map as input (-> no conflation with [OpenStreetMap](openstreetmap.org/))
    * provides visualization of the point cloud map in RVIZ2
   
* in the following, the sections are split between the two executables (however, keep in mind that kiss_icp_georef just provides a subset of the functions of lanelet2_osm)

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

</details>

<details>
<summary> <h2> 📈 Test Data </h2> </summary>

The test data in `/test` is from the EDGAR research vehicle (GPS trajectory). The SLAM poses were generated by [KISS-ICP](https://github.com/PRBonn/kiss-icp) in combination with [interactive SLAM](https://github.com/SMRT-AIST/interactive_slam). The lanelet2-map was created manually with [VectorMapBuilder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) by TieriV.

<details>
<summary> <h2> 🔧 Modules </h2> </summary>
</details>

Detailed documentation of the functionality behind the single modules can be found below.

1. [Geometric Alignment](doc/alignment.md)

2. [Preprocessing](doc/preprocessing.md)

3. [Matching](doc/matching.md)

4. [Conflation](doc/conflation.md)

5. [Analysis](doc/analysis.md)

</details>

<details>
<summary> <h2> 📇 Contact Info </h2> </summary>

[Maximilian Leitenstern](mailto:maxi.leitenstern@tum.de)
Institute of Automotive Technology,
School of Engineering and Design,
Technical University of Munich,
85748 Garching,
Germany

[Florian Sauerbeck](florian.sauerbeck@tum.de)
Institute of Automotive Technology,
School of Engineering and Design,
Technical University of Munich,
85748 Garching,
Germany
</details>

<details>
<summary> <h2> 📃 Citation </h2> </summary>
If you use this repository for any academic work, please cite our original paper:

```bibtex
@inproceedings{sauerbeck2023,
  title={Multi-LiDAR Localization and Mapping Pipeline for Urban Autonomous Driving},
  author={\textbf{Sauerbeck, Florian} and Kulmer, Dominik and Leitenstern, Maximilian and Weiss, Christoph and Betz, Johannes},
  booktitle={2023 IEEE Sensors},
  year={2023},
}
```
</details>
