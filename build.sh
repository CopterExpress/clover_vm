#!/bin/bash

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

echo "--- Building base image"

${PACKER} build -only=virtualbox-iso base_vm.json

echo "--- Building extended image"

${PACKER} build ros_ide_vm.json

echo "--- All done!"
