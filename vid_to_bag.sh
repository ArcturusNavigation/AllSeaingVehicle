#!/bin/bash

# Source directory for MOV files
SOURCE_DIR=~/Videos/mov

# Intermediate directory for resized MP4 files
MP4_DIR=~/Videos/bag_mp4

# Destination directory for bag files
BAG_DIR=~/AllSeaingVehicle/bags

# Create directories if they don't exist
mkdir -p "$MP4_DIR"
mkdir -p "$BAG_DIR"

# Convert .MOV files to .mp4 with specified width and height
for file in "$SOURCE_DIR"/*.MOV; do
  if [ ! -e "$file" ]; then continue; fi
  filename=$(basename -- "$file")
  filename_no_ext="${filename%.*}"
  ffmpeg -i "$file" -c:v libx264 -c:a copy -vf "scale=640:360" "$MP4_DIR/$filename_no_ext.mp4"
done

# Process the resized MP4 files to bag files
for mp4_file in "$MP4_DIR"/*.mp4; do
  if [ ! -e "$mp4_file" ]; then continue; fi
  mp4_filename=$(basename -- "$mp4_file")
  mp4_filename_no_ext="${mp4_filename%.*}"
  bag_filename="$mp4_filename_no_ext.bag"
  
  # Run the conversion using the Python script
  python3 src/utility/src/utility/mp4_to_bag.py "$mp4_file" "$bag_filename"

  # Move the generated bag file to the destination directory
  mv "$bag_filename" "$BAG_DIR/"
done

echo "All conversions completed."

