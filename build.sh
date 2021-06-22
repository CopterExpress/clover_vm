#!/bin/bash

set -e

# FIXME: Use system Packer if possible
#PACKER=$(which packer)

#if [ "x${PACKER}" == "x" ]; then
PACKER="./packer"
if [ ! -f ${PACKER} ]; then
    echo "Packer not found; downloading v1.5.4 from Hashicorp"
    wget --progress=dot:giga https://releases.hashicorp.com/packer/1.5.4/packer_1.5.4_darwin_amd64.zip
    unzip packer_1.5.4_darwin_amd64.zip
    rm packer_1.5.4_darwin_amd64.zip
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
GIT_DESCRIBE=$(git describe --tags)
VM_NAME="clover-devel_${GIT_DESCRIBE}.ova"
mv ./output-virtualbox-ovf/clover-devel.ova ./output-virtualbox-ovf/${VM_NAME}

echo "--- All done!"

if [ "${CI}" == "true" ]; then
    echo "Deploying to https://clovervm.ams3.digitaloceanspaces.com/${VM_NAME}"
fi
