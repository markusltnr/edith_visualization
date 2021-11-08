docker run --gpus all \
           --net=host \
	   --device /dev/dri \
           --privileged \
	   -v ~/v4r/edith:/root/share \
	   -v /usr/lib/nvidia/:/usr/lib/nvidia/ \
           -e QT_X11_NO_MITSHM=1 \
           -e DISPLAY=unix$DISPLAY \
	   -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
           -it --name vis_container vis /bin/bash
