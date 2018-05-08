#! /bin/bash
folder_name=$(date +%Y%m%d_%H_%M_%S)

echo ${folder_name}
cp sample.txt Python_codes
export LATEST_FOLDER_NAME=${folder_name}
