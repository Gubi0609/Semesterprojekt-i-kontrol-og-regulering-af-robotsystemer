#!/bin/sh

#
# Script to add the Quanser repository to the APT sources list.
#
# Copyright (c) 2021 Quanser Inc.

# Get the public key
wget --no-cache https://repo.quanser.com/keys/Quanser.pub

# Get the configuration files
wget --no-cache https://repo.quanser.com/debian/release/config/99-quanser
wget --no-cache https://repo.quanser.com/debian/release/config/quanser.sources

# Remove any old Quanser .list files
sudo rm -f /etc/apt/sources.list.d/quanser.list

# Remove any previous public Quanser keys
sudo apt-key del 44B9E16B84B393EE6BC9D3A7DB76774C160650A6
sudo apt-key del 72E89687D453563EAEBB7AD9D7288E94522541E1
sudo apt-key del B69848C0E97F9A9AF10BED165AB5E26373FDF160

# Move the files which tell APT where to find the repository
sudo mv 99-quanser /etc/apt/preferences.d/
sudo mv quanser.sources /etc/apt/sources.list.d/

# Add the public key for the repository
sudo gpg --dearmor --homedir /root --output /usr/share/keyrings/Quanser.gpg Quanser.pub
rm Quanser.pub