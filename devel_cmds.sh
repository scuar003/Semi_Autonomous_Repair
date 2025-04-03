#/bin/bash 

alias \
	build_app="sudo docker build -t ur16_app ." \
	clean_build_app="sudo docker build --no-cache -t ur16_app ." \
	build_image="ARCH=x86_64 ./tools/linuxdeploy-x86_64.AppImage --appdir AppDir --output appimage" \
	run_ur16_app="xhost +local:root && sudo docker run --rm --network host -e DISPLAY=$DISPLAY  -v $(pwd)/config:/config -v /tmp/.X11-unix:/tmp/.X11-unix -it ur16_app"


