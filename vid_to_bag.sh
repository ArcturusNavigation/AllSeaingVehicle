#!/bin/bash

# Define the destination directory
destination_directory="bags"

# Create the destination directory if it doesn't exist
mkdir -p "${destination_directory}"

for i in {1..6}
do
  # Run the conversion
  python3 src/utility/src/utility/mp4_to_bag.py ~/Videos/warehouse${i}_resize.mp4 warehouse${i}.bag

  # Move the generated bag file to the destination directory
  mv warehouse${i}.bag "${destination_directory}/"
done

