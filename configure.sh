#!/bin/bash

PLATFORM=$1

TX2_UNUSED="third_party/vision_opencv floatpi"
PI_UNUSED="third_party/avt_vimba_camera third_party/rtimulib_ros third_party/VINS-Fusion"
COMMON_UNUSED="third_party/vision_opencv"

usage()
{
  echo "Usage: ./configure.sh PLATFORM"
  echo "Available platforms: tx2, pi, common"
}

# Ensure platform has been given
if test -z "$PLATFORM"; then
  usage
  exit
fi

# Remove old catkin ignore files
for file in $(find . -name CATKIN_IGNORE ); do
  rm $file
done

# Add catkin ignore files to unused packages
if [ "$PLATFORM" = "tx2" ]; then
  for package in $TX2_UNUSED; do
    touch $package/CATKIN_IGNORE
  done
elif [ "$PLATFORM" = "pi" ]; then
  for package in $PI_UNUSED; do
    touch $package/CATKIN_IGNORE
  done
elif [ "$PLATFORM" = "common" ]; then
  for package in $COMMON_UNUSED; do
    touch $package/CATKIN_IGNORE
  done
fi
