xhost +
docker run -it --rm \
    --network=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --privileged \
    -v /home/maximilian/Documents/data:/workspace/data \
    tum_lanelet2_osm_fusion:v1
xhost -
