xhost +
docker run -it --rm \
    --network=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --privileged \
    -v /your/local/directory/data:/workspace/data \
    flexmap_fusion:v1
xhost -
