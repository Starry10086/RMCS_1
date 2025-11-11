import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction
)
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


# ========== 配置常量 ==========
ROBOT_NAME = 'robot_arm'
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']

# 启动延迟时间（秒）
CONTROLLER_DELAY = 2.0
ARM_CONTROLLER_DELAY = 1.0
MOVE_GROUP_DELAY = 2.0
RVIZ_DELAY = 3.0


def load_yaml(package_name, file_path):
    """加载YAML配置文件"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')
    pkg_share = get_package_share_directory('robot_arm')

    # 准备模型文件
    xacro_file = os.path.join(
        pkg_share, 'urdf', 'robot_arm.urdf.xacro'
    )
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_arm_base.urdf')

    if os.path.exists(xacro_file):
        # 使用xacro处理，传递mesh路径参数
        robot_description_content = Command([
            'xacro ', xacro_file,
            ' mesh_path:=', f'file://{pkg_share}/meshes/'
        ])
    else:
        # 回退到普通URDF
        with open(urdf_file, 'r') as f:
            robot_description_content = f.read()
        # 替换mesh路径为绝对路径
        robot_description_content = robot_description_content.replace(
            'package://robot_arm/meshes/',
            f'file://{pkg_share}/meshes/'
        )

    robot_description = {'robot_description': robot_description_content}

    # Controllers配置
    controllers_file = os.path.join(
        pkg_share, 'config', 'ros2_controllers.yaml'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}]
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Arm Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    # MoveIt配置
    srdf_file = os.path.join(pkg_share, 'config', 'robot_arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {
            'robot_description_semantic': f.read()
        }

    kinematics_yaml = load_yaml('robot_arm', 'config/kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml
    }

    joint_limits_yaml = load_yaml('robot_arm', 'config/joint_limits.yaml')
    robot_description_planning = {
        'robot_description_planning': {
            'joint_limits': (
                joint_limits_yaml.get('joint_limits', {})
                if joint_limits_yaml else {}
            )
        }
    }

    ompl_planning_yaml = load_yaml(
        'robot_arm', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/'
                'AddTimeOptimalParameterization '
                'default_planner_request_adapters/'
                'ResolveConstraintFrames '
                'default_planner_request_adapters/'
                'FixWorkspaceBounds '
                'default_planner_request_adapters/'
                'FixStartStateBounds '
                'default_planner_request_adapters/'
                'FixStartStateCollision '
                'default_planner_request_adapters/'
                'FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    if ompl_planning_yaml:
        ompl_planning_pipeline_config['move_group'].update(
            ompl_planning_yaml
        )

    moveit_controllers_yaml = load_yaml(
        'robot_arm', 'config/moveit_controllers.yaml'
    )

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Move Group
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
            {'use_sim_time': False},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # RViz
    rviz_config = os.path.join(pkg_share, 'rviz', 'moveit_config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            {'use_sim_time': False},
        ],
        condition=IfCondition(use_rviz)
    )

    # 启动顺序
    delay_controllers = TimerAction(
        period=CONTROLLER_DELAY,
        actions=[joint_state_broadcaster_spawner]
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=ARM_CONTROLLER_DELAY,
                    actions=[arm_controller_spawner]
                )
            ]
        )
    )

    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                TimerAction(
                    period=MOVE_GROUP_DELAY, actions=[move_group]
                )
            ]
        )
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                TimerAction(period=RVIZ_DELAY, actions=[rviz])
            ]
        )
    )

    return LaunchDescription([
        *declared_arguments,
        robot_state_publisher,
        controller_manager,
        delay_controllers,
        delay_arm_controller,
        delay_move_group,
        delay_rviz,
    ])