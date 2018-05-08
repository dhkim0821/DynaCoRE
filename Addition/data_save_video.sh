#! /bin/bash
PATH_PACKAGE="/home/hcrl/Repository/dynacore"
#PATH_PACKAGE=$(dirname "$(pwd)")
mercury_nodelet="/home/hcrl/ros/mercury_nodelet"

target_folder="/home/hcrl/MyCloud/Apptronik/Mercury_Test_2018/"
data_location=$PATH_PACKAGE

if [[ -z "${LATEST_FOLDER_NAME}" ]]; then
	echo "Error. The timestamped data folder does not exist. Make sure to source data_save_nas.sh first"
else
	echo "OK! The timestamped data folder environment name exists"
	if [ -f ${data_location}/experiment_data/*.mp4 ]; then 	# Check if the file exists
	    echo "OK! The mp4 files exist. Preparing to copy the files if it is possible"
	    echo " "
		if ! [[ `lsof ${data_location}/experiment_data/*.mp4 | grep python` ]] ; then # Check if we can safely access the file
			echo " "			
			echo "(ignore the lsof warnings)"
			echo "Copying mp4 files..."
			cp ${data_location}/experiment_data/*.mp4 ${target_folder}/${LATEST_FOLDER_NAME}/	
			if [ $? -eq 0 ]; then # Check if previous command was successful
			    echo "  Successfully copied the mp4 files!"
			    echo "  Moving local copy to trash."		    
			    # Move files to trash
				mv $PATH_PACKAGE/experiment_data/*.mp4 /home/$USER/.local/share/Trash/files/
				# Unset Folder name
				unset LATEST_FOLDER_NAME
			else
			    echo "Error. Could not copy the mp4 files"
			fi

		else
			echo " "
			echo "(ignore the lsof warnings)"			
		    echo "Error. The mp4 file is still being accessed by python"
		fi


	else
		echo "Error. No MP4 files exist"
	fi



fi

echo "...script finished."

