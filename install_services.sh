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
  echo "--enable disabled during development, please enable manually (due to conflicting services e.g. depth_control and thruster_control)"
  ENABLE=false
else
  ENABLE=false
fi


operations="start stop enable disable status"
sudo rm /etc/sudoers.d/floats

# Install
for service in $(ls etc/$PLATFORM/$MACHINE/services); do
  sudo cp etc/$PLATFORM/$MACHINE/services/$service /etc/systemd/system
  # Enable without sudo
  for op in $operations; do
    sudo echo "%float ALL=NOPASSWD: /bin/systemctl $op $service" >> /etc/sudoers.d/floats
  done
done

sudo systemctl daemon-reload

for script in $(ls etc/$PLATFORM/$MACHINE/scripts); do
  if [[ "$script" == *_user.sh ]]; then
     # Copy the scripts to ~/
     cp etc/$PLATFORM/$MACHINE/scripts/$script ~/
     chmod +x ~/$script
  else
     # Copy the script to /usr/local/bin
     sudo cp etc/$PLATFORM/$MACHINE/scripts/$script /usr/local/bin/
     sudo chmod +x /usr/local/bin/$script
  fi
done

# Optioanlly Enable
if [ $ENABLE = true ]; then
  for service in $( ls etc/$PLATFORM/$MACHINE/services ); do
    sudo systemctl enable $service
  done
fi


sudo cp tools/floats /usr/local/bin/
sudo chmod +x /usr/local/bin/floats
