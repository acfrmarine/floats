#!/bin/bash

PLATFORM=$1
MACHINE=$2

TX2_UNUSED="third_party/vision_opencv floatpi"
PI_UNUSED="third_party/avt_vimba_camera third_party/rtimulib_ros third_party/VINS-Fusion"
COMMON_UNUSED="third_party/vision_opencv"
D3_UNUSED="float_control"


usage()
{
  echo "Usage: ./configure.sh PLATFORM MACHINE"
  echo "Available platforms: d3, floatv1, float"
  echo "Available machines: tx2, pi, common"
}

# Ensure platform has been given
if test -z "$PLATFORM"; then
  usage
  exit
fi

# Ensure machine has been given
if test -z "$MACHINE"; then
  usage
  exit
fi

# Remove old catkin ignore files
for file in $(find . -name CATKIN_IGNORE ); do
  rm $file
done

# Add catkin ignore files to unused packages
if [ "$MACHINE" = "tx2" ]; then
  UNUSED_PACKAGES=$TX2_UNUSED
elif [ "$MACHINE" = "pi" ]; then
  UNUSED_PACKAGES=$PI_UNUSED
elif [ "$MACHINE" = "common" ]; then
  UNUSED_PACKAGES=$COMMON_USED
else
    echo "machine $MACHINE not available."
    exit
fi

if [ "$PLATFORM" = "d3" ]; then
    UNUSED_PACKAGES="$UNUSED_PACKAGES" "$D3_UNUSED"
elif [ "$PLATFORM" = "floatv1" ]; then
    UNUSED_PACKAGES="$UNUSED_PACKAGES" "$FLOATV1_UNUSED"
else
    echo "platform $PLATFORM not supported."
fi

for package in $UNUSED_PACKAGES; do
  touch $package/CATKIN_IGNORE
done
