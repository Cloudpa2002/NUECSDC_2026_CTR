import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Define LaunchConfigurations
    obj_num = LaunchConfiguration('obj_num', default='1')
    drone_id = LaunchConfiguration('drone_id', default='0')
    
    map_size_x = LaunchConfiguration('map_size_x', default='25.0')
    map_size_y = LaunchConfiguration('map_size_y', default='20.0')
    map_size_z = LaunchConfiguration('map_size_z', default='3.0')
    odom_topic = LaunchConfiguration('odom_topic', default='Odometry')
    planning_horizon = LaunchConfiguration('planning_horizon', default='4.0')
    
    # Declare launch arguments
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    planning_horizon_cmd = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon')

    # Map properties and simulation flags
    use_mockamap = LaunchConfiguration('use_mockamap', default='False')  # Should be string 'False' not boolean False
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    
    use_dynamic = LaunchConfiguration('use_dynamic', default='False')  # Should be string 'False' not boolean False
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')
    
    # # Map generator node
    # map_generator_node = Node(
    #     package='map_generator',
    #     executable='random_forest',
    #     name='random_forest',
    #     output='screen',
    #     parameters=[
    #         {'map/x_size': 26.0},
    #         {'map/y_size': 20.0},
    #         {'map/z_size': 3.0},
    #         {'map/resolution': 0.1},
    #         {'ObstacleShape/seed': 1.0},
    #         {'map/obs_num': 250},
    #         {'ObstacleShape/lower_rad': 0.5},
    #         {'ObstacleShape/upper_rad': 0.7},
    #         {'ObstacleShape/lower_hei': 0.0},
    #         {'ObstacleShape/upper_hei': 3.0},
    #         {'map/circle_num': 250},
    #         {'ObstacleShape/radius_l': 0.7},
    #         {'ObstacleShape/radius_h': 0.5},
    #         {'ObstacleShape/z_l': 0.7},
    #         {'ObstacleShape/z_h': 0.8},
    #         {'ObstacleShape/theta': 0.5},
    #         {'pub_rate': 1.0},
    #         {'min_distance': 0.8}
    #     ],
    #     condition=UnlessCondition(use_mockamap)
    # )

    # mockamap_node = Node(
    #     package='mockamap',
    #     executable='mockamap_node',
    #     name='mockamap_node',
    #     output='screen',
    #     remappings=[
    #         ('/mock_map', '/map_generator/global_cloud')
    #     ],
    #     parameters=[
    #         {'seed': 127},
    #         {'update_freq': 0.5},
    #         {'resolution': 0.1},
    #         {'x_length': PythonExpression(['int(', map_size_x, ')'])},
    #         {'y_length': PythonExpression(['int(', map_size_y, ')'])},
    #         {'z_length': PythonExpression(['int(', map_size_z, ')'])},
    #         {'type': 1},
    #         {'complexity': 0.05},
    #         {'fill': 0.12},
    #         {'fractal': 1},
    #         {'attenuation': 0.1}
    #     ],
    #     condition=IfCondition(use_mockamap)
    # )
    
    # Advanced parameters
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'airdetect_advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'cloud_registered',
            
            'cx': '321.04638671875',
            'cy': '243.44969177246094',
            'fx': '387.229248046875',
            'fy': '387.229248046875',
            'planning_horizon': planning_horizon,
            'use_distinctive_trajs': 'True',
            'flight_type': '1',
            'point_num': '4',
            'point0_x': '15.0',
            'point0_y': '0.0',
            'point0_z': '1.0',
            
            'point1_x': '-15.0',
            'point1_y': '0.0',
            'point1_z': '1.0',
            
            'point2_x': '15.0',
            'point2_y': '0.0',
            'point2_z': '1.0',
            
            'point3_x': '-15.0',
            'point3_y': '0.0',
            'point3_z': '1.0',
            
            'point4_x': '15.0',
            'point4_y': '0.0',
            'point4_z': '1.0',
        }.items()
    )
    
    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )
    
    # Ego control node
    cxr_egoctrl_v1_node = Node(
        package='ego_planner',
        executable='airdetect_egoctrl_v1', 
        name='airdetect_egoctrl_v1_node',
        output='screen'
    )
    
    # Simulator
    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': '0.0',
            'init_y_': '0.0',
            'init_z_': '0.0',
            'odometry_topic': odom_topic
        }.items()
    )
    
    ld = LaunchDescription()
    
    # Add all actions
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(planning_horizon_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    # ld.add_action(use_dynamic_cmd)
    # ld.add_action(use_mockamap_cmd)

    # ld.add_action(map_generator_node)
    # ld.add_action(mockamap_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(cxr_egoctrl_v1_node)
    ld.add_action(simulator_include)

    return ld
