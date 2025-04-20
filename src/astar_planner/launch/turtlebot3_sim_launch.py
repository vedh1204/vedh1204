import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directories
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    astar_dir = get_package_share_directory('astar_planner')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Map path
    map_path = os.path.join(astar_dir, 'maps', 'my_map.yaml')
    
    # Nav2 parameters file
    nav2_params_path = os.path.join(
        nav2_bringup_dir, 
        'params', 
        'nav2_params.yaml'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Map server with lifecycle management
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[
                {'yaml_filename': map_path},
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Lifecycle Manager for Map Server and AMCL
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']},
                {'use_sim_time': use_sim_time}
            ]
        ),
        
#        # Static Transform Publishers for transform chain
#        Node(
#            package='tf2_ros',
#            executable='static_transform_publisher',
#            name='static_transform_map_odom',
#            output='screen',
#            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
#        ),
#        
#            package='tf2_ros',
#            executable='static_transform_publisher',
#            name='static_transform_odom_base',
#            output='screen',
#            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
 #       ),
        
        # AMCL localization with parameters
        LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'alpha1': 0.2},
                {'alpha2': 0.2},
                {'alpha3': 0.2},
                {'alpha4': 0.2},
                {'alpha5': 0.2},
                {'beam_skip_distance': 0.5},
                {'beam_skip_threshold': 0.3},
                {'do_beamskip': False},
                {'global_frame_id': 'map'},
                {'lambda_short': 0.1},
                {'laser_likelihood_max_dist': 2.0},
                {'laser_max_range': 100.0},
                {'laser_min_range': -1.0},
                {'laser_model_type': 'likelihood_field'},
                {'max_beams': 60},
                {'max_particles': 2000},
                {'min_particles': 500},
                {'odom_frame_id': 'odom'},
                {'pf_err': 0.05},
                {'pf_z': 0.99},
                {'recovery_alpha_fast': 0.0},
                {'recovery_alpha_slow': 0.0},
                {'resample_interval': 1},
                {'robot_model_type': 'differential'},
                {'save_pose_rate': 0.5},
                {'sigma_hit': 0.2},
                {'tf_broadcast': True},
                {'transform_tolerance': 1.0},
                {'update_min_a': 0.2},
                {'update_min_d': 0.25},
                {'z_hit': 0.5},
                {'z_max': 0.05},
                {'z_rand': 0.5},
                {'z_short': 0.05},
                {'scan_topic': 'scan'},
                {'set_initial_pose': True},
                {'initial_pose.x': 0.0},
                {'initial_pose.y': -1.0},  # Fixed syntax here
                {'initial_pose.z': 0.0},
                {'initial_pose.yaw': 0.0}
            ]
        ),
        
        # Nav2 Controller Server
        LifecycleNode(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='',
            output='screen',
            parameters=[
                nav2_params_path,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Nav2 Planner Server
        LifecycleNode(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='',
            output='screen',
            parameters=[
                nav2_params_path,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Nav2 Behavior Tree Navigator
        LifecycleNode(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace='',
            output='screen',
            parameters=[
                nav2_params_path,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Nav2 Recovery Server
        LifecycleNode(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            namespace='',
            output='screen',
            parameters=[
                nav2_params_path,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Waypoint Follower
        LifecycleNode(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace='',
            output='screen',
            parameters=[
                nav2_params_path,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Lifecycle Manager for Navigation
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': [
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]},
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # A* planner node
        Node(
            package='astar_planner',
            executable='planner_node',
            name='astar_planner_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])



