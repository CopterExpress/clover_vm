#!/bin/bash

set -ex

# FIXME: Use system Packer if possible
#PACKER=$(which packer)

#if [ "x${PACKER}" == "x" ]; then
PACKER="./packer"
if [ ! -f ${PACKER} ]; then
    echo "Packer not found; downloading v1.5.4 from Hashicorp"
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        OS=linux
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        OS=darwin
    fi
    wget --progress=dot:giga https://releases.hashicorp.com/packer/1.5.4/packer_1.5.4_${OS}_amd64.zip
    unzip packer_1.5.4_${OS}_amd64.zip
    rm packer_1.5.4_${OS}_amd64.zip
fi

echo "--- Using Packer version $(${PACKER} --version)"

if [ ! -f output-virtualbox-iso/clover-devel.ova ]; then
    echo "--- Building base image"
    ${PACKER} build -only=virtualbox-iso base_vm.json || true
fi

echo "--- Building extended image"

${PACKER} build ros_ide_vm.json

echo "--- Marking the VM"

# if [[ $GITHUB_REF == refs/tags/*-rc* ]]; then
    # remove rc label
    # VERSION=${GITHUB_REF#refs/tags/}
    # VERSION=${VERSION/-rc*/}
if [[ $GITHUB_REF == refs/tags/* ]]; then
    VERSION=${GITHUB_REF#refs/tags/}
else
    VERSION=$(git describe --always)
fi

VM_NAME="clover-devel_${VERSION}.ova"
mv ./output-virtualbox-ovf/clover-devel.ova ./output-virtualbox-ovf/${VM_NAME}
ls -l output-virtualbox-ovf

echo "--- All done!"
