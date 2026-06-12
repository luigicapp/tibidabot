from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument #to declare arguments
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution #to use the arguments in the launch description(runtime configuration)
from launch_ros.actions import Node #to launch ROS2 nodes
from ament_index_python.packages import get_package_share_directory #to get the path of the package
def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true"
    )    

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value = "small_house",
        description='Name of the map to use for localization'
    )

    map_name = LaunchConfiguration("map_name")          #read the param at runtime
    use_sim_time = LaunchConfiguration("use_sim_time")  #read the param at runtime

    map_path = PathJoinSubstitution([
        get_package_share_directory("bumperbot_mapping"), #get the path of the package
        "maps", #we want to access the maps folder in the package
        map_name, #in the maps folder we have different maps, we want to access the one specified by the map_name argument
        "map.yaml" #the map file is called map.yaml
    ])
    #will start a ros2 node (we imported the launch_ros.actions.Node class) with the following parameters:
    nav2_map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path}, #the map server
            {"use_sim_time": use_sim_time} #use sim time if the argument is set to true
        ]
    )

    lifecycle_nodes = ["map_server"] #the lifecycle manager will manage these nodes, in this case only the map server node but we can add other nodes if we want to

    #the lifecycle manager will automatically transition the nodes 
    #to the active state when they are launched, it will also manage the 
    #lifecycle of the nodes (e.g. if a node crashes it will try to restart it)
    #(needed because it would be unconfortable to have to manually transition the nodes 
    #to the active state every time we launch the localization stack, especially if we have 
    #multiple nodes to manage)
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name = "lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes}, #the lifecycle manager will manage the nodes specified in the lifecycle_nodes list
            {"use_sim_time": use_sim_time}, #use sim time if the argument is set to true
            {"autostart": True} #the lifecycle manager will automatically transition the nodes to the active state when they are launched   
        ]
    )

    #take a list of all the nodes in the package and add them to the launch description
    #(the so called "actions" in ROS2) and return it
    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        nav2_map_server_node,
        nav2_lifecycle_manager
    ])