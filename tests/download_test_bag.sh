#!/bin/bash

# Download script for test ROS bag file
# This downloads a sample bag file for testing bag2rrd

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BAG_URL="https://download.ifi.uzh.ch/rpg/drone_racing_data/race_1.bag"
BAG_FILE="$SCRIPT_DIR/data/race_1.bag"

echo "Downloading ROS bag file for testing..."
echo "URL: $BAG_URL"
echo "Destination: $BAG_FILE"

if [ -f "$BAG_FILE" ]; then
    echo "File already exists: $BAG_FILE"
    echo "Skipping download. Remove the file if you want to re-download."
    exit 0
fi

# Create data directory if it doesn't exist
mkdir -p "$SCRIPT_DIR/data"

# Download with progress bar
curl -L -o "$BAG_FILE" --progress-bar "$BAG_URL"

echo "Download complete!"
echo "File size: $(du -h "$BAG_FILE" | cut -f1)"
echo ""
echo "You can now test bag2rrd with:"
echo "  cargo run -- inspect $BAG_FILE"
echo "  cargo run -- convert $BAG_FILE test_output.rrd"