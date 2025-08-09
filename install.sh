#!/bin/bash

MODULE_NAME="laputil"
MODULE_VERSION="1.0"  # Change to match dkms.conf PACKAGE_VERSION
CUR_DIR=$(pwd)
DKMS_DIR="/usr/src/${MODULE_NAME}-${MODULE_VERSION}"

# Copy source files to /usr/src/module-version directory
echo "Copying source files to ${DKMS_DIR}..."
sudo rm -rf "${DKMS_DIR}"
sudo mkdir -p "${DKMS_DIR}"
sudo cp -r "${CUR_DIR}/." "${DKMS_DIR}/"
if [ $? -ne 0 ]; then
  echo "Failed to copy source files."
  exit 1
fi

echo "Adding DKMS module..."
sudo dkms add -m "${MODULE_NAME}" -v "${MODULE_VERSION}" -k "$(uname -r)"
if [ $? -ne 0 ]; then
  echo "DKMS add failed."
  exit 1
fi

echo "Building DKMS module..."
sudo dkms build -m "${MODULE_NAME}" -v "${MODULE_VERSION}" -k "$(uname -r)"
if [ $? -ne 0 ]; then
  echo "DKMS build failed."
  exit 1
fi

echo "Installing DKMS module..."
sudo dkms install -m "${MODULE_NAME}" -v "${MODULE_VERSION}" -k "$(uname -r)"
if [ $? -ne 0 ]; then
  echo "DKMS install failed."
  exit 1
fi

echo "DKMS module installed successfully."

echo "Current DKMS status:"
dkms status | grep "${MODULE_NAME}"


