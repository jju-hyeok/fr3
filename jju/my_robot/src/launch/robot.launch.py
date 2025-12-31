import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception:
        return None


def generate_launch_description():
    # --- launch args ---
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.16.0.2',
        description='Hostname or IP address of the FR3 controller (FCI).'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot (optional).'
    )

    robot_ip = LaunchConfiguration('robot_ip')
    namespace = LaunchConfiguration('namespace')

    # --- pkg_share (네 패키지명으로 수정) ---
    pkg_share = get_package_share_directory('my_robot')  # ← 여기를 네 패키지명으로 변경

    # --- robot_description (URDF from franka_description FR3) ---
    fr3_xacro = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    robot_description_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        fr3_xacro, ' ',
        'hand:=true ',                 # 실제 그리퍼 장착이면 true
        'ros2_control:=true ',
        'robot_ip:=', robot_ip, ' ',
        'use_fake_hardware:=false ',
        'fake_sensor_commands:=false'
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_cmd, value_type=str)
    }

    # --- robot_description_semantic (SRDF from franka_description FR3) ---
    fr3_srdf_xacro = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro'
    )
    robot_semantic_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        fr3_srdf_xacro, ' ',
        'hand:=true'
    ])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_semantic_cmd, value_type=str)
    }

    # --- MoveIt configs (use official franka_fr3_moveit_config) ---
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    if ompl_yaml:
        ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        'my_robot', 'config/moveit_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # --- ros2_control (use official controllers file) ---
    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'config', 'fr3_ros_controllers.yaml'
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    )

    # --- spawn controllers (arm + js broadcaster) ---
    load_controllers = []
    for controller in ['fr3_arm_controller', 'joint_state_broadcaster', 'franka_robot_state_broadcaster']:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager', PathJoinSubstitution([namespace, 'controller_manager'])
                ],
                output='screen'
            )
        )

    # --- Franka gripper (실기) ---
    franka_gripper_node = Node(
        package='franka_gripper',
        executable='franka_gripper_node',
        name='franka_gripper',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'franka_gripper_node.yaml'),  # 네가 만든 YAML
            {'robot_ip': robot_ip}
        ]
    )

    # --- robot_state_publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )

    # --- MoveGroup ---
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # --- RViz (official config) ---
    rviz_cfg = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'rviz', 'moveit.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_cfg],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # --- joint_state_publisher (arm + gripper merge) ---
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{
            'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],  # ← 토픽명 수정
            'rate': 30
        }],
    )

    # === Launch order (권장 순서) ===
    return LaunchDescription(
        [
            robot_ip_arg,
            namespace_arg,

            # 1) ros2_control + controllers
            ros2_control_node,
        ] + load_controllers + [
            # 2) gripper node (액션 서버 준비)
            franka_gripper_node,

            # 3) publishers / planning / viz
            robot_state_publisher,
            run_move_group_node,
            rviz_node,
            joint_state_publisher,
        ]
    )

