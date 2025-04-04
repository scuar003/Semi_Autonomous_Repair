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

# Define the default config file path (adjust this path as needed).
DEFAULT_CONFIG="$HOME/Semi_Autonomous_Repair/config/params.yaml"

# Check if the default config exists. If not, try the current working directory, then prompt.
if [ -f "$DEFAULT_CONFIG" ]; then
    CONFIG_FILE="$DEFAULT_CONFIG"
elif [ -f "$(pwd)/config/params.yaml" ]; then
    CONFIG_FILE="$(pwd)/config/params.yaml"
else
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
terminator -e "bash -c 'docker run --rm --init --network host -e DISPLAY=\"$DISPLAY\" -v \"$CONFIG_DIR\":/config -v /tmp/.X11-unix:/tmp/.X11-unix -it ur16_app; exec bash'"
