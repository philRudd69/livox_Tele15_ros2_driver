"""
loop over all configuration-possibilies.
this script will automatically alter the livox_lidar_config.json
config file for the tele-15, launch the tele-15_rviz_launch.py file with the correct parameters,
record for 10s and store the rosbag in the correct directory.
Afterwards the analysis script will be automatically applied to the rosbag, before the script
continues with the next configuration.
"""

import json
import subprocess
import os
from threading import Thread
import time
from rosbag2_py import Recorder, RecordOptions, StorageOptions

config_file = "/home/markus/Development/sim_poc_nvidia_omniverse/ws_tele15_ROS2/src/livox_ros2_driver/config/livox_lidar_config.json"
ros_recordings_dir = "/home/markus/Development/sim_poc_nvidia_omniverse/ws_tele15_ROS2/ROS2_recordings"

return_modes = [0,1,2]
coordinates = [0,1]
xfer_formats = [0,1,2,4]
recording_duration_s = 10.0  #[s]

def record(recorder, storage_opts, record_opts):
    try:
        recorder.record(storage_opts, record_opts)
    except KeyboardInterrupt:
        pass

def setRos2bagRecorder(rec_path=".", duration_sec=10, ):

    print(f"Recording to {rec_path} for {duration_sec} seconds...")

    storage_opts = StorageOptions(uri=rec_path)
    record_opts = RecordOptions()
    record_opts.all_topics = True
    recorder = Recorder()

    print("Start recording")
    record_thread = Thread(target=record, args=(recorder, storage_opts, record_opts))
    record_thread.start()
    time.sleep(duration_sec)
    print("Stopping recording")
    recorder.cancel()
    record_thread.join()
    print("Done!")

def loopOverConfigs():
    for return_mode in return_modes:
        for coordinate in coordinates:
            for xfer_format in xfer_formats:
                # return_mode and coordinate can be changed in the config.json file:
                if os.path.isfile(config_file):
                    with open(config_file, 'r+') as file:
                        data = json.load(file)
                        data["lidar_config"][0]["return_mode"] = return_mode
                        data["lidar_config"][0]["coordinate"] = coordinate
                        file.seek(0)
                        json.dump(data, file, indent=4)
                        file.truncate()
                
                # create empty directory to store the rosbag in
                new_folder_name = f'ret_{return_mode}_crd_{coordinate}_xfer_{xfer_format}'
                if os.path.isdir(ros_recordings_dir):
                    new_folder_path = os.join(ros_recordings_dir, new_folder_name)
                    if not os.path.exists(new_folder_path):
                        os.mkdir(new_folder_path)

                    # xfer_format is a ROS2 parameter of the driver, we have to set it in the CLI
                    launch_delay_s = 5.0
                    shutdown_buffer_s = 2.0
                    # launch the driver but set timeout to end the non-ending driver-process after that timeout
                    return_code = subprocess.call(f"ros2 launch livox_ros2_driver tele15_rviz_launch.py xfer_format:={xfer_format}", timeout=launch_delay_s + recording_duration_s + shutdown_buffer_s)
                    # wait 5s to allow the nodes to launch
                    time.sleep(launch_delay_s)
                    # launch ros2 bag record -a
                    setRos2bagRecorder(rec_path=new_folder_path, duration_sec=recording_duration_s)   # should return after recording_duration_s

                    
                    