#/bin/bash 

alias \
	build_app="sudo docker build -t ur16_app ." \
	clean_build_app="sudo docker build --no-cache -t ur16_app ." \
	run_ur16_app="xhost +local:root && sudo docker run --rm --network host -e ROBOT_IP=192.168.56.101 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it ur16_app"


