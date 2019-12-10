#!/bin/bash
set -e

PLATFORM=$1

usage()
{
  echo "Usage: ./install_services.sh PLATFORM [--enable]"
  echo "Available platforms: tx2, pi"
}

# Ensure platform has been given
if test -z "$PLATFORM"; then
  usage
  exit
fi

if [ "$2" == "--enable" ]; then
  ENABLE=true
else
  ENABLE=false
fi

# install and optionally enable services
if [ "$PLATFORM" = "tx2" ]; then
  for service in $( ls etc/tx2/services ); do
    sudo cp etc/tx2/services/$service /etc/systemd/system/
  done
  for script in $( ls etc/tx2/scripts ); do
    sudo cp etc/tx2/scripts/$script /usr/local/bin/
  done
  sudo systemctl daemon-reload
  if [ $ENABLE = true ]; then
    for service in $( ls etc/tx2/services ); do
      sudo systemctl enable $service
    done
  fi
elif [ "$PLATFORM" = "pi" ]; then
  for service in $( ls etc/pi/services ); do
    sudo cp etc/pi/services/$service /etc/systemd/system/
  done
  for script in $( ls etc/pi/scripts ); do
    if [ $script = "ros_screen_user.sh" ]; then
      cp etc/pi/scripts/$script ~/
    else
      sudo cp etc/pi/scripts/$script /usr/local/bin/
    fi
  done
  sudo systemctl daemon-reload
  if [ $ENABLE = true ]; then
    for service in $( ls etc/pi/services ); do
      sudo systemctl enable $service
    done
  fi
fi