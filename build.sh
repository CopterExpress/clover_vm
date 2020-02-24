#!/bin/bash

PACKER=$(which packer)

if [ "x${PACKER}" == "x" ]; then
    echo "Packer not found; downloading v1.5.4 from Hashicorp"
    wget https://releases.hashicorp.com/packer/1.5.4/packer_1.5.4_linux_amd64.zip
    unzip packer_1.5.4_linux_amd64.zip
    rm packer_1.5.4_linux_amd64.zip
    PACKER="./packer"
fi

echo "--- Using Packer version $(${PACKER} --version)"

echo "--- Building base image"

${PACKER} build base_vm.json

echo "--- Building extended image"

${PACKER} build ros_ide_vm.json

echo "--- All done!"
