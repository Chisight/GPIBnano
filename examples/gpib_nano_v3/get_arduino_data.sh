#!/bin/bash

# --- Configuration ---
# The serial device file.
DEVICE="/dev/ttyUSB0"
# The baud rate and other settings for the serial port.
STTY_SETTINGS="115200 cs8 -cstopb -parenb raw -echo"
# The command string to send to the device. The script adds the newline.
COMMAND_STRING="*init 22,*write F5T3,*listen"


# --- Pre-flight Checks ---
# Ensure the device file exists and we can write to it.
if [ ! -w "$DEVICE" ]; then
    echo "Error: Device '$DEVICE' not found or not writable." >&2
    echo "Check device connection and permissions (e.g., add user to 'dialout' group)." >&2
    exit 1
fi


# --- Main Logic ---
# Open the serial device for both reading and writing on file descriptor 3.
# This keeps the connection active and prevents the device from resetting.
exec 3<> "$DEVICE"

# Configure the serial port settings.
stty -F "$DEVICE" $STTY_SETTINGS

# Send the command string, followed by a newline, to the device.
printf "%s\n" "$COMMAND_STRING" >&3

read -r -t 3 line <&3

echo $line 

# --- Cleanup ---
# Close the file descriptor.
exec 3>&-
