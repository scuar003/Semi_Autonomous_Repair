#/bin/bash 

alias \
	build_app="docker build -t ur16_app ." \
	clean_build_app="docker build --no-cache -t ur16_app ." \
	run_ur16_app="xhost +local:root && docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ur16_app"


