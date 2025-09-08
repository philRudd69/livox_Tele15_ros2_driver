import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


################### user configure parameters for ros2 start ###################
xfer_format   = 4    # 0-LivoxPointcloud2(PointXYZRTL), 1-customized pointcloud format, 2-StandardPointcloud2(PointXYZI), 4-LivoxPointcloud2(PointXYZTPRRTL)
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar,1-hub
publish_freq  = 10.0 # freqency of publish,1.0,2.0,5.0,10.0,etc
output_type   = 0    # 0-Output to ROS, 1-Output to ROS bag file
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

sensor_config = 'livox_lidar_config.json'
rviz_config = 'livox_lidar.rviz'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
################### user configure parameters for ros2 end #####################


def generate_launch_description():
    
    livox_ros2_parameters = [
        DeclareLaunchArgument('xfer_format', default_value=str(xfer_format)),
        DeclareLaunchArgument('multi_topic', default_value=str(multi_topic)),
        DeclareLaunchArgument('data_src', default_value=str(data_src)),
        DeclareLaunchArgument('publish_freq', default_value=str(publish_freq)),
        DeclareLaunchArgument('output_data_type', default_value=str(output_type)),
        DeclareLaunchArgument('frame_id', default_value=frame_id),
        DeclareLaunchArgument('lvx_file_path', default_value=lvx_file_path),
        DeclareLaunchArgument('cmdline_input_bd_code', default_value=cmdline_bd_code),
        DeclareLaunchArgument('user_config_path', default_value=str(os.path.join(cur_config_path, sensor_config))),
        DeclareLaunchArgument('rviz_config_path', default_value=str(os.path.join(cur_config_path, rviz_config)))
    ]

    livox_driver = Node(
        package='livox_ros2_driver',
        executable='livox_ros2_driver_node',
        name='livox_lidar_publisher',
        output='screen',
        #prefix=['valgrind --leak-check=full --show-leak-kinds=all'],
        parameters=[
            {"xfer_format": LaunchConfiguration('xfer_format')},
            {"multi_topic": LaunchConfiguration('multi_topic')},
            {"data_src": LaunchConfiguration('data_src')},
            {"publish_freq": LaunchConfiguration('publish_freq')},
            {"output_data_type": LaunchConfiguration('output_data_type')},
            {"frame_id": LaunchConfiguration('frame_id')},
            {"lvx_file_path": LaunchConfiguration('lvx_file_path')},
            {"user_config_path": LaunchConfiguration('user_config_path')},
            {"cmdline_input_bd_code": LaunchConfiguration('cmdline_input_bd_code')}
        ]
        )

    livox_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', LaunchConfiguration('rviz_config_path')]
        )

    return LaunchDescription([
        *livox_ros2_parameters,
        livox_driver,
        livox_rviz,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
