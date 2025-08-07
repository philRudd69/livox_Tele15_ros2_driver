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
import time
from pathlib import Path
from udp_packet_loss_analysis import udpPacketLossAnalysis
# from threading import Thread
# from rosbag2_py import Recorder, RecordOptions, StorageOptions

config_file = "/home/markus/Development/sim_poc_nvidia_omniverse/ws_tele15_ROS2/src/livox_ros2_driver/config/livox_lidar_config.json"
ros_recordings_dir = "/home/markus/Development/sim_poc_nvidia_omniverse/ws_tele15_ROS2/ROS2_recordings"

return_modes = [0, 1, 2]
coordinates = [0, 1]
xfer_formats = [0, 1, 2, 4]
recording_duration_s = 10.0  # [s]


''' # This only works if rosbag2_py is available, which means ROS2 Humble or higher.
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
'''


def recordWithCLI(path: str):
    print("Starting ros2 record")
    process = subprocess.Popen(["ros2", "bag", "record", "-a"], stdout=subprocess.PIPE, cwd=f"{path}")
    time.sleep(recording_duration_s)
    print("Stopping ros2 record")
    process.send_signal(subprocess.signal.SIGINT)  # kill -9 deos not work for ros2 bag record. it will not exit the process cleanly.


def adaptConfigFile(return_mode, coordinate):
    # return_mode and coordinate can be changed in the config.json file:
    if os.path.isfile(config_file):
        with open(config_file, 'r+') as file:
            data = json.load(file)
            data["lidar_config"][0]["return_mode"] = return_mode
            data["lidar_config"][0]["coordinate"] = coordinate
            file.seek(0)
            json.dump(data, file, indent=4)
            file.truncate()
        print(f"Adapted contents of config file {config_file}")
    else:
        raise RuntimeError(f"Config file does not exist: {config_file}")


def createROSbagDirectory(return_mode, coordinate, xfer_format):
    # create empty directory to store the rosbag in
    new_folder_name = f'ret_{return_mode}_crd_{coordinate}_xfer_{xfer_format}'
    if os.path.isdir(ros_recordings_dir):
        new_folder_path = os.path.join(ros_recordings_dir, new_folder_name)
        if not os.path.exists(new_folder_path):
            os.mkdir(new_folder_path)
            print(f"Created new directory {new_folder_path}")
        return new_folder_path
    else:
        raise RuntimeError(f"Directory for storing ROSbag files does not exist: {ros_recordings_dir}")


def startROSdriverAndRecording(xfer_format, path):
    # xfer_format is a ROS2 parameter of the driver, we have to set it in the CLI
    launch_delay_s = 10.0
    shutdown_delay_s = 2.0
    # launch the driver but set timeout to end the non-ending driver-process after that timeout
    print("Starting launch file")
    process = subprocess.Popen(["ros2", "launch", "livox_ros2_driver", "tele15_launch.py", f"xfer_format:={xfer_format}"], stdout=subprocess.PIPE)  # , timeout=launch_delay_s + recording_duration_s + shutdown_buffer_s)
    # wait 5s to allow the nodes to launch
    time.sleep(launch_delay_s)
    # setRos2bagRecorder(rec_path=new_folder_path, duration_sec=recording_duration_s)   # should return after recording_duration_s  # API not available for ROS2 foxy
    recordWithCLI(path)
    time.sleep(shutdown_delay_s)
    print("Stopping launch file")
    subprocess.run(["kill", "-9", f"{process.pid}"])


def changeConfigAndRecord(return_mode, coordinate, xfer_format):
    print(f"Starting test of configuration return_mode={return_mode}, coordinate={coordinate}, xfer_format={xfer_format}")
    adaptConfigFile(return_mode, coordinate)
    path = createROSbagDirectory(return_mode, coordinate, xfer_format)
    startROSdriverAndRecording(xfer_format, path)


def plotHistograms(return_mode, coordinate, xfer_format, obj):
    # get recording path
    folder_name = f'ret_{return_mode}_crd_{coordinate}_xfer_{xfer_format}'
    folder_path = Path(os.path.join(ros_recordings_dir, folder_name))
    # find the rosbag recording folders in this directory
    if folder_path.exists():
        if folder_path.is_dir():  # user input is directory of ros-recroding folders
            folders = list(folder_path.glob("rosbag2*"))  # find the files
            for folder in folders:
                if xfer_format == 0:
                    obj.processRecording(path=folder,
                                         spherical_points_available=False,
                                         detect_missing_points=False,
                                         detect_time_jumps_between_frames=True,
                                         benchmark_driver=False,
                                         do_plotting=False)
                elif xfer_format == 4:
                    if coordinate == 0:  # special case where the ROS2 driver switches to xfer_format 0
                        obj.processRecording(path=folder,
                                             spherical_points_available=False,
                                             detect_missing_points=False,
                                             detect_time_jumps_between_frames=True,
                                             benchmark_driver=False,
                                             do_plotting=False)
                    if coordinate == 1:
                        obj.processRecording(path=folder,
                                             spherical_points_available=True,
                                             detect_missing_points=False,
                                             detect_time_jumps_between_frames=True,
                                             benchmark_driver=False,
                                             do_plotting=False)
                if xfer_format == 1 or xfer_format == 2:
                    print("Analysis-Tools not yet available.")


def loopOverConfigs(func, **obj):
    for return_mode in return_modes:
        for coordinate in coordinates:
            for xfer_format in xfer_formats:
                func(return_mode, coordinate, xfer_format, **obj)


if __name__ == "__main__":
    # loopOverConfigs(func=changeConfigAndRecord)
    analysis = udpPacketLossAnalysis()
    loopOverConfigs(func=plotHistograms, obj=analysis)
