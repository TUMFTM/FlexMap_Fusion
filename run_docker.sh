xhost +
docker run -it --rm \
    --network=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --privileged \
    # Mount your local directory to the container
    -v /your/local/directory:/workspace/data \
    flexmap_fusion:v1
xhost -
