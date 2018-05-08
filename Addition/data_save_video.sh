#! /bin/bash
#PATH_PACKAGE="/home/hcrl/Repository/dynacore"
PATH_PACKAGE=$(dirname "$(pwd)")
mercury_nodelet="/home/hcrl/ros/mercury_nodelet"

target_folder="/home/hcrl/MyCloud/Apptronik/Mercury_Test_2018/"
data_location=$PATH_PACKAGE

if [[ -z "${LATEST_FOLDER_NAME}" ]]; then
	echo "data folder doesn't exist"
else
	echo "Copying movie files if it exists..."
	cp ${data_location}/experiment_data/*.mp4 ${target_folder}/${LATEST_FOLDER_NAME}/
	echo "Finished copying the video"
	rm $PATH_PACKAGE/experiment_data/*.mp4	
	unset LATEST_FOLDER_NAME	
fi


