#!/usr/bin/env bash

sudo apt update
sudo apt install -y build-essential make perl
sudo mount -o loop,ro /home/clever/VBoxGuestAdditions.iso /mnt
sudo /mnt/VBoxLinuxAdditions.run
sudo umount /mnt
