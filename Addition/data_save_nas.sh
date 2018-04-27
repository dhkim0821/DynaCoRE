#! /bin/bash
#PATH_PACKAGE="/home/hcrl/Repository/dynacore"
PATH_PACKAGE=$(dirname "$(pwd)")

folder_name=$(date +%Y%m%d_%H_%M_%S)
target_folder="/home/hcrl/MyCloud/Apptronik/Mercury_Test_2018/"
data_location=$PATH_PACKAGE
mkdir -p ${target_folder}/${folder_name}
mkdir -p ${target_folder}/${folder_name}/Config

cp ${data_location}/DynaController/Mercury_Controller/MercuryTestConfig/* ${target_folder}/${folder_name}/Config/
cp ${data_location}/experiment_data/* ${target_folder}/${folder_name}/
cp ${data_location}/experiment_data/* ${data_location}/experiment_data_check/


# Move experiment data to the other folder to plot data

# Remove every data for the next experiment
rm $PATH_PACKAGE/experiment_data/*

exit 0
