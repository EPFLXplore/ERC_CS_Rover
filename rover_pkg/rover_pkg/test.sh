#!/bin/bash

# Check if the topic name argument is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <topic_name>"
    exit 1
fi

TOPIC_NAME="$1"

echo "bababa"

# Execute the ros2 command in the background
output=$(ros2 topic bw "$TOPIC_NAME" 2>&1 &)

# Give the command a moment to start
sleep 2

# Extract the bandwidth number from the output
BANDWIDTH=$(echo "$output" | grep -oP '[0-9.]+ KB/s' | head -n 1)

echo "bab"
# Kill the background process
kill $!

# Print the bandwidth
echo "Bandwidth: $BANDWIDTH"