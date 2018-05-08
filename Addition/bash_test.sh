#! /bin/bash
folder_name=$(date +%Y%m%d_%H_%M_%S)

echo ${folder_name}
export LATEST_FOLDER_NAME=${folder_name}

if [ -f ./*.txt ]; then
    echo "Text file exists"
fi

echo "hello"
if [ $? -eq 0 ]; then #Check if previous command was successful
    echo "Success!"
else
    echo "Fail"
fi

if [[`lsof /home/hcr/Repository/dynacore/experiment_data/*.mp4 | grep python`]]; then
	echo "File is being accessed"
else 
	echo "File is not being accessed"
fi