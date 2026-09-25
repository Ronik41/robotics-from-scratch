"""ROS graph for M3. The existing supervisor owns Gazebo and process cleanup."""
from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]


def setup(context):
    urdf = Path(LaunchConfiguration('urdf').perform(context)).read_text()
    tree = ET.fromstring(urdf)
    # Read dimensions from the same expanded model used by Gazebo, not duplicated constants.
    left = tree.find("joint[@name='left_wheel_joint']/origin").attrib['xyz'].split()
    right = tree.find("joint[@name='right_wheel_joint']/origin").attrib['xyz'].split()
    radius = float(tree.find("link[@name='left_wheel']/collision/geometry/cylinder").attrib['radius'])
    sensors = LaunchConfiguration("sensors").perform(context) == "true"
    firmware = LaunchConfiguration("firmware").perform(context) == "true"
    nodes = [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='rover_bridge',
             parameters=[{'config_file': str(ROOT / ('simulation/config/bridge-firmware.yaml' if firmware else 'simulation/config/bridge-sensors.yaml' if sensors else 'simulation/config/bridge.yaml')), 'use_sim_time': True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf, 'use_sim_time': True, 'publish_frequency': 50.0}]),
        ExecuteProcess(cmd=['python3', str(ROOT / 'src/rover_odometry/node.py'), '--ros-args',
                            '-p', 'use_sim_time:=true', '-p', f'wheel_radius:={radius}',
                            '-p', f'wheel_separation:={float(left[1])-float(right[1])}',
                            '-p', f'axle_x:={float(left[0])}']),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', str(ROOT / ('simulation/config/sensors.rviz' if sensors else 'simulation/config/rover.rviz'))],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
        ExecuteProcess(cmd=['xterm', '-T', 'Rover teleop: i forward / comma reverse / j,l turn / k stop',
                            '-fa', 'monospace', '-fs', '11', '-geometry', '80x18+20+560', '-e', 'ros2', 'run', 'teleop_twist_keyboard',
                            'teleop_twist_keyboard', '--ros-args', '-p', 'speed:=0.2', '-p', 'turn:=0.5',
                            '-p', 'use_sim_time:=true', *(['-p', 'stamped:=true', '-p', 'frame_id:=base_link'] if firmware else [])], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    if sensors:
        nodes.insert(3, ExecuteProcess(cmd=['python3', str(ROOT / 'src/rover_sensors/node.py'),
                                           '--ros-args', '-p', 'use_sim_time:=true']))
    if firmware:
        nodes.insert(0, ExecuteProcess(cmd=['python3', str(ROOT / 'src/rover_control/node.py'),
                                          '--ros-args', '-p', 'use_sim_time:=true',
                                          '-p', f'wheel_radius:={radius}',
                                          '-p', f'wheel_separation:={float(left[1])-float(right[1])}']))
    if firmware:
        nodes.append(ExecuteProcess(cmd=['xterm', '-T', 'M5 motor safety diagnostics',
                                         '-fa', 'monospace', '-fs', '10', '-geometry', '82x25+730+450',
                                         '-e', 'python3', str(ROOT/'scripts/control-monitor.py')],
                                    condition=IfCondition(LaunchConfiguration('rviz'))))
    # A dead bridge/estimator must not leave a superficially healthy launch behind.
    return [*nodes, *(RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='Required ROS component exited'))])) for node in nodes[:6 if firmware else 5 if sensors else 4])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('urdf'), DeclareLaunchArgument('firmware', default_value='false'), DeclareLaunchArgument('sensors', default_value='false'), DeclareLaunchArgument('rviz', default_value='false'),
                              OpaqueFunction(function=setup)])
