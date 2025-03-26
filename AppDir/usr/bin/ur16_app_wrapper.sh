#!/bin/bash
# Ensure Docker is installed.
if ! command -v docker &> /dev/null; then
    zenity --error --text="Docker is not installed. Please install Docker first."
    exit 1
fi



# Ensure DISPLAY is set.
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

# Try to use the config file in the current working directory.
CONFIG_FILE="$(pwd)/config/params.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    # If not found, prompt the user with a file-selection dialog.
    CONFIG_FILE=$(zenity --file-selection --title="Select config file" --filename="$HOME/" --file-filter="*.yaml")
    if [ -z "$CONFIG_FILE" ]; then
        zenity --error --text="No config file selected. Exiting."
        exit 1
    fi
fi

CONFIG_DIR=$(dirname "$CONFIG_FILE")

# Launch the GUI parameter editor.
python3 "$(dirname "$0")/edit_params.py" "$CONFIG_FILE"


# Open a new terminal (using Terminator) to run the Docker container.
# The command below starts Terminator, executes the docker run command, and then keeps the terminal open.
terminator -e "bash -c 'docker run --rm --init --network host -e DISPLAY=\"$DISPLAY\" -v \"$CONFIG_DIR\":/config -v /tmp/.X11-unix:/tmp/.X11-unix -it ur16_app; exec bash'"
