#/bin/bash 

alias \
	dockerfile="vim Dockerfile" \
	build_app="docker build -t ur16_app ." \
	run_ur16_app="docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ur16_app"


