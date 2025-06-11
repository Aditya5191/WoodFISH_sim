import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    #same name as robot name in xacro file
    robotXacroName='AUV'
    #name of package.. must be same as the pkg name as it will be used to define paths
    namePackage='robot_description'

    modelFileReativePath='model/robot.xacro'

    #if wanna create a custom empty world uncomment this(will need to creare empty_world.world file)
    worldFileReativePath='worlds/water_world.sdf'

    #abs path to model
    pathModelFile=os.path.join(get_package_share_directory(namePackage),modelFileReativePath)

    #abs path to world
    pathWorldFile=os.path.join(get_package_share_directory(namePackage),worldFileReativePath)

    #get the robot description from xacro file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    #launch file from gazebo
    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))
    #this is the launch description

    #if using own world uncomment this
    gazeboLaunch = IncludeLaunchDescription(
    gazebo_rosPackageLaunch,
    launch_arguments={
        'gz_args': f'-r -v 4 {pathWorldFile}',
        'on_exit_shutdown': 'true'
    }.items()
)

    #if using default world
    #gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'],'on_exit_shutdown': 'true'}.items())

    spawnModelNodeGazebo=Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',robotXacroName,
            '-topic','robot_description',
            ],
            output='screen',
    )

    nodeRobotStatePublisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robotDescription,
                     'use_sim_time': True}],
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_params.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
            ],
    )


    # thruster_controller_node = Node(
    #     package='controller',
    #     executable='thruster_controller',
    #     name='thruster_controller',
    #     output='screen',
    # )



    LaunchDescriptionObject=LaunchDescription()
    #add all the nodes to the launch description
    LaunchDescriptionObject.add_action(gazeboLaunch)
    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    LaunchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    # LaunchDescriptionObject.add_action(thruster_controller_node)  
    
    #return the launch description
    return LaunchDescriptionObject

