xhost +
docker run -it --rm \
    --network=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --privileged \
    # -v /home/maximilian/Documents/gitlab/scanmatcher/tools/tum_lanelet2_osm_fusion:/workspace/src/tum_lanelet2_osm_fusion \
    -v /home/maximilian/Documents/data:/workspace/data \
    gitlab.lrz.de:5005/sensor-fusion-slam/scanmatcher/lanelet2_osm_fusion_img

    bash -c "ros2 launch tum_lanelet2_osm_fusion kiss_icp_georef.launch.py traj_path:=/workspace/data/EDGAR/Route_1/Route_1_GPS.txt poses_path:=/workspace/data/EDGAR/Route_1/route1_pose_kitti.txt pcd_path:=/workspace/data/EDGAR/Route_1/route1_map.pcd"
xhost -
