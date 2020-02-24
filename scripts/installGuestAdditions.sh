#!/usr/bin/env bash

sudo apt update
sudo apt install -y build-essential make perl
# virtualbox is still terrible, let's target vmware
sudo apt install -y open-vm-tools
