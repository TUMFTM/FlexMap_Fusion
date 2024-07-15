# FlexMap Fusion
[![Linux](https://img.shields.io/badge/os-linux-blue.svg)](https://www.linux.org/)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
[![ROS2humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/index.html)

## Overview
![image](doc/img/pipeline_offline_mapping.png)
*Offline Mapping Pipeline developed at the Institute of Automotive Technology*
* ROS2 package for the improvement/extension of lanelet2 maps with data from [OpenStreetMap](openstreetmap.org/) and their georeferencing with GNSS data
* The package is designed for the application in an offline mapping pipeline developed at the Institute of Automotive Technology of the Technical University of Munich
* The package consists of the three modules "Map Alignment", "Map Conflation" and "Georeferencing", that are explained in more detail in the image below and the documentation of the single modules
![image](doc/img/conflation_tool.png)
*Overview of the single modules of the FlexMap Fusion tool*
* The following functionalities are included
   * georeferencing of the lanelet2 map based on the transformation calculated by the vehicle ego trajectory from GNSS and the SLAM-trajectory
   * fusion of semantic information from [OpenStreetMap](openstreetmap.org/) into the lanelet2 map (generated from point cloud map data)
   * modular expandability to include additional semantic information
   * possibility to apply georeferencing step on the point cloud map resulting from the SLAM process and used for the generation of the lanelet map
   * extensive visualization within RVIZ2

> [!NOTE]  
> If you're more interested in georeferencing a 3D pointcloud map, check out [FlexCloud](https://github.com/TUMFTM/FlexCloud), a further development of the georeferencing stage supporting fast 2D and 3D rubber-sheeting with Delauny-Triangulations.

<details>
<summary> <h2> 🐋 Docker Setup </h2> </summary>

### Package Design
This package is designed as a standalone ROS2 package. It was developed with ROS2 humble. For easier handling of dependencies, a docker environment is provided that sets up everything and builds the package.
As the package is designed for use in combination with [Autoware](https://github.com/autowarefoundation/autoware), the source code can also be build within the [Autoware](https://github.com/autowarefoundation/autoware) docker environment.

### Setup

1. Clone the repository by running
   ```bash
   git clone git@github.com:TUMFTM/FlexMap_Fusion.git
   ```
2. Go to the root directory of the repository
   ```bash
   cd FlexMap_Fusion/
   ```
3. Build the docker image
   ```bash
   ./docker/build_docker.sh  
   ```
4. Run the container and mount your data by appending the directory containing your data:
   ```bash
   ./docker/run_docker.sh /your/local/directory/data
   ```
</details>

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
<details> 
<summary> <h3> 📄 lanelet2_osm </h2> </summary>
   
1. Necessary input parameters:
   - `traj_path` => path to GPS trajectory of the vehicle (format: txt-file with lat, lon)
   - `poses_path` => path to SLAM trajectory of the vehicle (KITTI-format, trajectories don't have to be synchronized over time)
   - `map_path` => path to lanelet2 map corresponding to trajectories (map can have missing elements/attributes, only when using node `lanelet2_osm`)
   - `out_path` => path to save the modified lanelet map (DEFAULT: /lanelet2_map.osm, only when using node `lanelet2_osm`)
   - if you want to georeference the point cloud map corresponding to the lanelet2 map with the same set of control points:
      - set the parameter `transform_pcd` in the config file to `true`
      - adjust the path to the point cloud map (parameter `pcd_path`)
      - the georeferenced point cloud map will be saved in the current working directory (if you'd like to specify a different path, see `kiss_icp_georef`
2. Start the package
   - it is recommended to directly use the provided ROS launch file as it starts the package itself and the visualization in RVIZ:
   - replace the filepaths and run the following command inside the docker container:

   ```bash
       ros2 launch flexmap_fusion lanelet2_osm.launch.py traj_path:=<path-to-GPS-trajectory> poses_path:=<path-to-SLAM-trajectory>  map_path:=<path-to-lanelet-map> out_path:=<path-to-save-output-map>
   ```
   - the launch file directly links to the corresponding parameter file in `/config/`.
   - To directly run the package with the provided test files from the Docker root directory, use the following command:
   ```bash
        ros2 launch flexmap_fusion lanelet2_osm.launch.py traj_path:=./src/flexmap_fusion/test/route_1_GPS.txt poses_path:=./src/flexmap_fusion/test/route1_pose_kitti.txt  map_path:=./src/flexmap_fusion/test/lanelet2_route_1.osm out_path:=lanelet2_map_georef.osm
   ```

3. Select control points
   - after the trajectories are loaded and the target trajectory is aligned to the master trajectory by the Umeyama algorithm, you are asked in the command window to select control points for the rubber-sheet transformation (the amount of points can be configured in the config file).
   - select the desired points using the `Publish Point` button in RVIZ and follow the instructions in the command window.
4. Inspect results
   - results of the rubber-sheet transformation & lanelet map are visualized
   - see table for explanation of single topics

| Topic | Description |
| ----------- | ----------- |
| `/lof/map/osm_map_markers` | Street network downloaded from [OpenStreetMap](openstreetmap.org/). |
| `/lof/map/ll_map_markers` | Lanelet2-map enriched with attributes from [OpenStreetMap](openstreetmap.org/); Lanelets are colorized based on agreement between adjacent lanelets and lanes-tag of [OpenStreetMap](openstreetmap.org/). |
| `/lof/map/ll_map_new_markers` | Lanelet2-map enriched with attributes from [OpenStreetMap](openstreetmap.org/); Lanelets that are likely to be wrong were removed by the module "Deletion of Lanelet Fragments". |
| `/lof/traj/traj_master_markers` | Master trajectory -> to be defined in config-file (usually GNSS-trajectory) |
| `/lof/traj/traj_target_markers` | original target rajectory (usually SLAM-trajectory) |
| `/lof/traj/traj_align_markers` | target trajectory aligned to master with [Umeyama](https://web.stanford.edu/class/cs273/refs/umeyama.pdf)-algorithm |
| `/lof/traj/traj_rs_markers` | target trajectory after [rubber-sheet](https://www.tandfonline.com/doi/abs/10.1559/152304085783915135)-transformation based on control points |
| `/lof/rs/geom_markers` | geometric information from rubber-sheeting (control points and constructed triangles) |
| `/clicked_point` | last 2 selected points by user to indicate chosen control points |
| `/lof/confl/geom_markers` | geometric information regarding conflation process (collapsed lanelet-map, buffers, matches) |

   - Inspect results and modify parameters if desired.
5. Manually finalize lanelet map
   - open a manual editor for lanelet2 maps (e.g. [VectorMapBuilder](https://tools.tier4.jp/feature/vector_map_builder_ll2/)) in parallel to RVIZ
   - import the exported map from `out_path`
   - close gaps in lanelet map and correct other mistakes based on visualization of map agreement with [OpenStreetMap](openstreetmap.org/) in RVIZ

</details>

<details> 
<summary> <h3> 📄 kiss_icp_georef </h2> </summary>
   
1. Necessary input parameters:
   - `traj_path` => path to GPS trajectory of the vehicle (format: txt-file with lat, lon)
   - `poses_path` => path to SLAM trajectory of the vehicle (KITTI-format, trajectories don't have to be synchronized over time)
   - `pcd_path` => path to pcd map corresponding to poses trajectory
   - `pcd_out_path` => path to saved the georeferenced point cloud map (DEFAULT: /pcd_map_georef.pcd)
2. Start the package
   - it is recommended to directly use the provided ROS launch file that starts the package itself and the visualization in RVIZ:

   ```bash
       ros2 launch flexmap_fusion kiss_icp_georef.launch.py traj_path:=<path-to-GPS-trajectory> poses_path:=<path-to-SLAM-trajectory>  pcd_path:=<path-to-pcd-map> pcd_out_path:=<path-to-save-pcd-map>
   ```

   - the launch file directly links to the corresponding parameter file in `/config/`.

3. Select control points
   - after the trajectories are loaded and the target trajectory is roughly aligned to the master trajectory you are asked in the command window to select control points for the rubber-sheet transformation (the amount of points can be configured in the config file).
   - select the desired points using the `Publish Point` button in RVIZ and follow the instructions in the console.
4. Inspect results
   - results of the rubber-sheet transformation & the resulting, transformed point cloud map are visualized.
   - see table for explanation of single topics

| Topic | Description |
| ----------- | ----------- |
| `/lof/traj/traj_master_markers` | Master trajectory -> to be defined in config-file (either GNSS- or SLAM trajectory) |
| `/lof/traj/traj_target_markers` | original target rajectory -> depending on selected master trajectory (either GNSS- or SLAM trajectory) |
| `/lof/traj/traj_align_markers` | target trajectory aligned to master with [Umeyama](https://web.stanford.edu/class/cs273/refs/umeyama.pdf) transformation or [PCL ICP](https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html) |
| `/lof/traj/traj_rs_markers` | target trajectory after [rubber-sheet](https://www.tandfonline.com/doi/abs/10.1559/152304085783915135)-transformation |
| `/lof/rs/geom_markers` | geometric information from rubber-sheeting (control points and constructed triangles) |
| `/clicked_point` | last 2 selected points by user to indicate chosen control point |
| `/lof/rs/pcd_map` | transformed point cloud map (only when using `kiss_icp_georef`) |

   - Inspect results and modify parameters if desired.
</details>

* the parameter files for both executables are located in `/config`
* see the comments within the single `.param.yaml`-files for detailed explanations on the parameters
* to modify the parameters inside the container and view their current value, python-executables are provided:
   * to view the current value of a parameter, run
   ```bash
      get_param.py <config_file_name> <param_name>
   ```
   inside the container. 
   * to modify a parameter, run
   ```bash
      config_param.py <config_file_name> <param_name> <param_value> <param_type>
   ```
   inside the container. Supported parameter types are `str`, `int`, `float` and `bool`. See the current value for the right choice, otherwise the node will crash.
   * Example:
   Set amount of control points for rubber-sheeting:
   ```bash
      config_param.py lanelet2_osm.param.yaml rs_num_controlPoints 10 int
   ```
   Check effect:
   ```bash
      get_param.py lanelet2_osm.param.yaml rs_num_controlPoints
   ```

</details>

<details>
<summary> <h2> 📈 Test Data </h2> </summary> (GPS trajectory). The SLAM poses were generated by [KISS-ICP](https://github.com/PRBonn/kiss-icp) in combination with [interactive SLAM](https://github.com/SMRT-AIST/interactive_slam). The lanelet2-map was created manually with [VectorMapBuilder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) by TieriV.

</details>

<details>
<summary> <h2> 🔧 Modules </h2> </summary>

Detailed documentation of the functionality behind the single modules can be found below.

1. [Geometric Alignment](doc/alignment.md)

2. [Preprocessing](doc/preprocessing.md)

3. [Matching](doc/matching.md)

4. [Conflation](doc/conflation.md)

5. [Georeferencing](doc/georef.md)

6. [Analysis](doc/analysis.md)

</details>

<details>
<summary> <h2> 📇 Contact Info </h2> </summary>

[Maximilian Leitenstern](mailto:maxi.leitenstern@tum.de),
Institute of Automotive Technology,
School of Engineering and Design,
Technical University of Munich,
85748 Garching,
Germany

[Florian Sauerbeck](mailto:florian.sauerbeck@tum.de),
Institute of Automotive Technology,
School of Engineering and Design,
Technical University of Munich,
85748 Garching,
Germany

[Dominik Kulmer](mailto:dominik.kulmer@tum.de),
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
@misc{leitenstern2024flexmap,
      title={FlexMap Fusion: Georeferencing and Automated Conflation of HD Maps with OpenStreetMap}, 
      author={Maximilian Leitenstern and Florian Sauerbeck and Dominik Kulmer and Johannes Betz},
      year={2024},
      eprint={2404.10879},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
</details>
