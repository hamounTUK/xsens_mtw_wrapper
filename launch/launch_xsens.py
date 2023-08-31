from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    desired_update_rate_launch_arg = DeclareLaunchArgument('desired_update_rate', default_value=TextSubstitution(text='60'))



    return LaunchDescription([
        desired_update_rate_launch_arg,
        Node(
            package='hiros_xsens_mtw_wrapper',
            executable='hiros_xsens_mtw_wrapper_node',
            name='hiros_xsens_mtw_wrapper_node',
            output='screen',
            parameters=[
                {"xsens_mtw_node_required"  : False},
                {"node_name"                : "xsens_mtw"},
                {"tf_prefix"                : ""},
                {"desired_update_rate"      : 60},
                {"desired_radio_channel"    : 19},
                {"reset_initial_orientation": False},
                {"enable_custom_labeling"   : False},
                {"synchronize"              : True},
                {"sync_policy"              : "skip_partial_frames"},
                {"publish_mimu_array"       : True},
                {"publish_imu"              : True},
                {"publish_mag"              : True},
                {"publish_euler"            : False},
                {"publish_free_acceleration": False},
                {"publish_pressure"         : False},
                {"publish_tf"               : False},

            ]
        )
    ])