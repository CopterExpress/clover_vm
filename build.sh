#!/bin/bash

set -e

# FIXME: Use system Packer if possible
#PACKER=$(which packer)

#if [ "x${PACKER}" == "x" ]; then
PACKER="./packer"
if [ ! -f ${PACKER} ]; then
    echo "Packer not found; downloading v1.5.4 from Hashicorp"
    wget --progress=dot:giga https://releases.hashicorp.com/packer/1.5.4/packer_1.5.4_linux_amd64.zip
    unzip packer_1.5.4_linux_amd64.zip
    rm packer_1.5.4_linux_amd64.zip
fi

echo "--- Using Packer version $(${PACKER} --version)"

if [ ! -f output-virtualbox-iso/clover-devel.ova ]; then
    echo "--- Building base image"
    ${PACKER} build -only=virtualbox-iso base_vm.json || true
fi

echo "--- Building extended image"

${PACKER} build ros_ide_vm.json

echo "--- Marking the VM"

GIT_REV=$(git rev-parse --short HEAD)
mv ./output-virtualbox-ovf/clover-devel.ova ./output-virtualbox-ovf/clover-devel_v0.20+${GIT_REV}.ova 

echo "--- All done!"
