#!/bin/bash

# Define paths and filenames
WORKSPACE_DIR=~/Main/programming/ros2_ws
HRWROS_DIR="$WORKSPACE_DIR/src/hello_world_ros2_edx/hrwros_week2/hrwros_assignments"
HRWROS_XACRO="$HRWROS_DIR/hrwros_week2_assignment/urdf/hrwros_assignment1.xacro"

DEBUG_DIR=~/Downloads/debug_hrwros_support
HRWROS_URDF="$DEBUG_DIR/hrwros.urdf"  # Store URDF in the Downloads directory
HRWROS_PDF="$DEBUG_DIR/robot.pdf"    # Store PDF in the Downloads directory

# Change to the desired output directory
cd "$DEBUG_DIR"

# Ensure the cleanup only affects our target files in the Downloads directory
echo "Cleaning up previous URDF and PDF files in $DEBUG_DIR..."
find "$DEBUG_DIR" -type f \( -path "${HRWROS_URDF}" -o -path "${HRWROS_PDF}" \) -exec rm -f {} +

# Generate URDF from XACRO
echo "Generating URDF from XACRO in $DEBUG_DIR..."
if xacro "$HRWROS_XACRO" > "$HRWROS_URDF"; then
  echo "URDF file generated successfully."
else
  echo "Error: Failed to generate URDF from XACRO."
  exit 1
fi

# Check the URDF file for correctness
echo "Validating URDF file in $DEBUG_DIR..."
if check_urdf "$HRWROS_URDF"; then
  echo "URDF file is valid."
else
  echo "Error: URDF file validation failed."
  exit 1
fi

# Generate and attempt to display the robot model graph
echo "Generating graph from URDF in $DEBUG_DIR..."
if urdf_to_graphviz "$HRWROS_URDF"; then
  echo "Graph generated. Attempting to display in $DEBUG_DIR..."
  if command -v evince &> /dev/null; then
    evince "$HRWROS_PDF" &
  else
    echo "Warning: 'evince' is not installed. Please open $HRWROS_PDF manually."
  fi
else
  echo "Error: Failed to generate or open graph from URDF."
  exit 1
fi

ros2 run tf2_tools view_frames

echo "Process completed successfully."


# Change back to the original directory if needed
# cd -