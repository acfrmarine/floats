#!/bin/bash
set -e

PLATFORM=$1
MACHINE=$2
usage()
{
  echo "Usage: ./install_services.sh PLATFORM MACHINE [--enable]"
  echo "Available platforms: d3, floatv1"
  echo "Available platforms: tx2, pi"
}

# Ensure platform has been given
if test -z "$PLATFORM"; then
  usage
  exit
fi

# Ensure platform has been given
if test -z "$MACHINE"; then
  usage
  exit
fi


if [ "$3" == "--enable" ]; then
  ENABLE=true
else
  ENABLE=false
fi

# Install
for service in $(ls etc/$PLATFORM/$MACHINE/services); do
  sudo cp etc/$PLATFORM/$MACHINE/$service /etc/systemd/system
done
sudo systemctl daemon-reload

# Optioanlly Enable
if [ $ENABLE = true ]; then
  for service in $( ls etc/$PLATFORM/$MACHINE/services ); do
    sudo systemctl enable $service
  done
fi
