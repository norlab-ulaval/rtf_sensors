#! /bin/bash
screen_name="altimeter"
# Check if the screen session named "altimeter" exists
if screen -list | grep -q $screen_name; then
    # If it exists, send the quit command to the screen session
    screen -r -S $screen_name -X quit
fi
echo "Starting ${screen_name}"
command="ros2 launch rtf_sensors dps310.launch.xml"
screen -S $screen_name -dm bash -c "bash --init-file <(echo \"history -s '$command'; bash -i -c '$command'\")"
