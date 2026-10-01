# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Gazebo Sim startup loop for the NAO model (Task 1 + Task 3B joint states).

Starts Gazebo Sim with a minimal world, injects the Gazebo resource path so
that ``model://nao_meshes/...`` resolves, spawns ``nao_simple.sdf`` and,
optionally, starts ``robot_state_publisher`` with ``nao_simple.urdf``.

Task 3B adds the minimal joint-state link:

    Gazebo joint-state publisher (raw /nao/joint_states_raw)
      -> ros_gz_bridge (raw JointState)
      -> nao_gz_sim/joint_state_converter (validates, clears effort)
      -> /joint_states (single public output)
      -> robot_state_publisher (TF consumer only)

Task 3C adds:

* the /clock bridge with the frozen CLOCK QoS profile (via a config file);
* a minimal body-pose converter publishing /body_pose (canonical ``base_link``
  world pose from the Gazebo ``nao`` model pose, frame_id=world).

Task 3D adds:

* a native Gazebo IMU sensor on ``base_link`` (canonical
  ``ImuTorsoAccelerometer_frame`` pose), bridged to a raw Imu and converted to
  the frozen ``/imu/data`` with ``frame_id=ImuTorsoAccelerometer_frame``.

Task 3E adds:

* one native Gazebo contact sensor on each foot link ``l_ankle`` / ``r_ankle``
  monitoring the frozen main collision ``l_ankle_collision`` /
  ``r_ankle_collision``, bridged as internal raw ``Contacts`` and converted to
  the frozen ``/left_foot/contact`` and ``/right_foot/contact``
  (``std_msgs/msg/Bool``, ground_plane-only判定).

Task 3F adds:

* ``world_tf_converter`` publishing exactly one dynamic transform
  ``world -> base_link`` on ``/tf`` from the Gazebo ``nao`` model world pose
  (simulation stamp preserved); ``robot_state_publisher`` remains the unique
  authority for the canonical URDF joint/fixed TF tree.

A /clock bridge is started so that ``use_sim_time`` works for every ROS node.
No controller, no joint command, no ros2_control, no /tf bridge and no other
sensor bridge are started here.
"""

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Gazebo Sim executable resolved once; falls back to the bare name so the
# error is reported by the launch system if it is missing from PATH.
GZ_EXECUTABLE = shutil.which('gz') or 'gz'


def generate_launch_description():
    pkg_nao_gz_sim = get_package_share_directory('nao_gz_sim')
    pkg_nao_description = get_package_share_directory('nao_description')
    pkg_nao_meshes = get_package_share_directory('nao_meshes')

    world_path = os.path.join(pkg_nao_gz_sim, 'worlds', 'nao_world.sdf')
    sdf_path = os.path.join(pkg_nao_description, 'urdf', 'nao_simple.sdf')
    urdf_path = os.path.join(pkg_nao_description, 'urdf', 'nao_simple.urdf')
    clock_bridge_config = os.path.join(pkg_nao_gz_sim, 'config', 'clock_bridge.yaml')

    # Task 4 Phase 4A: the actuator system plugin is installed next to the other
    # package libraries. Gazebo needs it on GZ_SIM_SYSTEM_PLUGIN_PATH.
    pkg_prefix = get_package_prefix('nao_gz_sim')
    phase4a_plugin_dir = os.path.join(pkg_prefix, 'lib', 'nao_gz_sim')

    # Gazebo resolves ``model://nao_meshes/...`` by searching every entry of
    # GZ_SIM_RESOURCE_PATH for ``<entry>/nao_meshes/...``. The required entry is
    # therefore the parent of the nao_meshes package share directory.
    resource_root = os.path.dirname(pkg_nao_meshes)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = os.pathsep.join(
        [entry for entry in [resource_root, existing_resource_path] if entry])
    existing_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = os.pathsep.join(
        [entry for entry in [phase4a_plugin_dir, existing_plugin_path] if entry])

    # Task 4 Phase 4A/4B: the runtime model used when enable_phase4a or
    # enable_phase4b is set. It is the canonical model (identical link/joint/
    # sensor/interface definitions) plus exactly one actuator plugin element; the
    # canonical SDF file itself is never modified, so
    # tools/check_task3a_contract.py keeps passing. The instance-unique temporary
    # path avoids concurrent-launch overwrite (Phase 4A review P3 #3).
    with open(sdf_path, 'r') as sdf_file:
        canonical_sdf_text = sdf_file.read()

    def _phase4_model_spawn(context, *args, **kwargs):
        enable4a = LaunchConfiguration('enable_phase4a').perform(context)
        enable4b = LaunchConfiguration('enable_phase4b').perform(context)
        delay = float(LaunchConfiguration('spawn_delay').perform(context))
        world = LaunchConfiguration('world_name').perform(context)
        spawn_z_value = LaunchConfiguration('spawn_z').perform(context)
        actions = []
        if enable4a == 'true' or enable4b == 'true':
            p_gain = LaunchConfiguration('actuator_p_gain').perform(context)
            max_velocity = LaunchConfiguration('actuator_max_velocity').perform(context)
            hold = LaunchConfiguration('actuator_hold_on_start').perform(context)
            plugin_xml = (
                '\n  <plugin filename="libnao_phase4a_actuator.so" '
                'name="nao_gz_sim::NaoPhase4aActuator">\n'
                '    <joint_command_topic>/joint_command</joint_command_topic>\n'
                '    <p_gain>%s</p_gain>\n'
                '    <max_velocity>%s</max_velocity>\n'
                '    <hold_on_start>%s</hold_on_start>\n'
                '  </plugin>\n' % (p_gain, max_velocity, hold))
            fd, model_file = tempfile.mkstemp(
                prefix='nao_phase4_runtime_', suffix='.sdf')
            insert_at = canonical_sdf_text.rfind('</model>')
            with os.fdopen(fd, 'w') as runtime_sdf_file:
                runtime_sdf_file.write(
                    canonical_sdf_text[:insert_at] + plugin_xml +
                    canonical_sdf_text[insert_at:])
            actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
                ExecuteProcess(cmd=['rm', '-f', model_file])])))
        else:
            model_file = sdf_path
        actions.append(TimerAction(
            period=delay,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-world', world, '-file', model_file, '-name', 'nao',
                           '-x', '0.0', '-y', '0.0', '-z', spawn_z_value],
                output='screen')]))
        return actions

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    gui = LaunchConfiguration('gui')
    use_rsp = LaunchConfiguration('use_rsp')
    physics_engine = LaunchConfiguration('physics_engine')
    enable_phase4a = LaunchConfiguration('enable_phase4a')
    enable_phase4b = LaunchConfiguration('enable_phase4b')
    enable_phase4d = LaunchConfiguration('enable_phase4d')
    phase4a_run_trace = LaunchConfiguration('phase4a_run_trace')

    gz_env = {
        'GZ_SIM_RESOURCE_PATH': gz_resource_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
    }

    # Gazebo Sim 8 selects its physics engine from --physics-engine; the SDF
    # <physics type="..."> attribute is ignored. The default is DART.
    # The optional gz_seed is forwarded verbatim to ``gz sim --seed`` (P0-a P1
    # physics-seed wiring). When gz_seed is empty the command is byte-identical
    # to the historical one (no --seed), so the default behaviour is unchanged.
    def _gz_sim_actions(context, *args, **kwargs):
        engine = physics_engine.perform(context)
        seed = LaunchConfiguration('gz_seed').perform(context)
        base = [GZ_EXECUTABLE, 'sim', '-v', '4', '-r']
        extra = (['--seed', str(seed)] if seed else [])
        tail = ['--physics-engine', engine, world_path]
        return [
            ExecuteProcess(
                cmd=base + extra + tail,
                output='screen',
                additional_env=gz_env,
                condition=IfCondition(gui),
            ),
            ExecuteProcess(
                cmd=base + ['-s'] + extra + tail,
                output='screen',
                additional_env=gz_env,
                condition=UnlessCondition(gui),
            ),
        ]

    # Spawn of the runtime/canonical model is resolved at launch time by
    # _phase4_model_spawn (see above) so that actuator tuning args are honoured.

    # Task 3C: /clock bridge with the frozen CLOCK QoS profile
    # (BEST_EFFORT + VOLATILE + KEEP_LAST depth 1) via a config file, so that
    # use_sim_time is driven by Gazebo simulation time.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args',
                   '-p', 'config_file:=' + clock_bridge_config,
                   '-p', 'use_sim_time:=true'],
        output='screen',
    )

    # Task 3B: internal raw joint-state bridge. The raw topic is an
    # implementation detail and is NOT part of the Task 3A public contract; the
    # only public joint-state output is /joint_states below.
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/nao/joint_states_raw@sensor_msgs/msg/JointState[gz.msgs.Model'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Task 3B: validate the raw bridge output and publish the frozen public
    # /joint_states (effort cleared, names/order checked, sim stamp preserved).
    joint_state_converter = Node(
        package='nao_gz_sim',
        executable='joint_state_converter',
        parameters=[{
            'use_sim_time': True,
            'raw_topic': '/nao/joint_states_raw',
            'output_topic': '/joint_states',
            'qos_depth': 10,
        }],
        output='screen',
    )

    # Task 3C: canonical body pose from the Gazebo model world pose. Subscribes
    # directly to the Gazebo topic via gz-transport (the Pose_V -> ROS bridge
    # drops names and header stamps, so it is not used) and publishes the frozen
    # /body_pose. It does NOT publish /tf.
    body_pose_converter = Node(
        package='nao_gz_sim',
        executable='body_pose_converter',
        parameters=[{
            'use_sim_time': True,
            'model_name': 'nao',
            'pose_topic': '/world/nao_world/pose/info',
            'output_topic': '/body_pose',
        }],
        output='screen',
    )

    # Task 3D: internal raw IMU bridge. The raw topic is an implementation
    # detail; the only public IMU output is /imu/data below.
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/nao/imu_raw@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Task 3D: fix the frame_id to the canonical ImuTorsoAccelerometer_frame,
    # preserve the Gazebo stamp, check finite values, normalize orientation and
    # publish the frozen /imu/data. It does NOT publish /tf.
    imu_converter = Node(
        package='nao_gz_sim',
        executable='imu_converter',
        parameters=[{
            'use_sim_time': True,
            'raw_topic': '/nao/imu_raw',
            'output_topic': '/imu/data',
            'frame_id': 'ImuTorsoAccelerometer_frame',
            'qos_depth': 10,
        }],
        output='screen',
    )

    # Task 3E: internal raw contact bridges. The raw topics are implementation
    # details; the only public contact outputs are /left_foot/contact and
    # /right_foot/contact below. Task 4 Phase 4E-0 adds the
    # ``enable_contact_converter`` switch (default true) so a dedicated negative
    # test can disable the contact path at runtime WITHOUT changing the default
    # Task 3 behaviour.
    contact_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/nao/left_foot_contact_raw@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/nao/right_foot_contact_raw@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_contact_converter')),
        output='screen',
    )

    # Task 3E: map the scoped collision names to the two frozen Bool topics.
    # Only the foot main collision against ground_plane yields true; the
    # converter uses Gazebo simulation time (via /clock) for freshness. It does
    # NOT publish /tf and never uses a controller.
    contact_converter = Node(
        package='nao_gz_sim',
        executable='contact_converter',
        parameters=[{
            'use_sim_time': True,
            'left_raw_topic': '/nao/left_foot_contact_raw',
            'right_raw_topic': '/nao/right_foot_contact_raw',
            'update_rate': 100.0,
            'stale_after': 0.05,
        }],
        condition=IfCondition(LaunchConfiguration('enable_contact_converter')),
        output='screen',
    )

    # Task 3F: the single dynamic world -> base_link publisher on /tf. It
    # publishes exactly one transform (from the Gazebo nao model world pose,
    # Gazebo simulation stamp preserved) and nothing else. robot_state_publisher
    # below is the unique authority for the canonical URDF joint chain and fixed
    # transforms. The Pose_V -> TF bridge is deliberately not used.
    world_tf_converter = Node(
        package='nao_gz_sim',
        executable='world_tf_converter',
        parameters=[{
            'use_sim_time': True,
            'model_name': 'nao',
            'pose_topic': '/world/nao_world/pose/info',
        }],
        output='screen',
    )

    # TF authority: standard robot_state_publisher with the approved canonical
    # URDF description. Deliberately NOT nao_state_publisher. It only consumes
    # /joint_states and is the unique authority for the canonical URDF dynamic
    # joint chain (/tf) and fixed transforms (/tf_static). It does NOT publish
    # world -> base_link: that single dynamic transform is published by
    # world_tf_converter above. The body-pose node only publishes /body_pose.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        condition=IfCondition(use_rsp),
    )

    # Task 4 Phase 4A: read-only closed-loop trace recorder. Writes the raw
    # /clock, /joint_command and /joint_states samples and the response summary.
    # It publishes nothing. Started only with enable_phase4a:=true.
    phase4a_evidence_dir = LaunchConfiguration('phase4a_evidence_dir')
    phase4a_trace = Node(
        package='nao_gz_sim',
        executable='phase4a_trace.py',
        parameters=[{
            'use_sim_time': True,
            'target_joint': LaunchConfiguration('phase4a_joint'),
            'output_jsonl': PathJoinSubstitution(
                [phase4a_evidence_dir, 'command_state_trace.jsonl']),
            'output_summary': PathJoinSubstitution(
                [phase4a_evidence_dir, 'phase4a_summary.json']),
            'record_duration_sec': ParameterValue(
                LaunchConfiguration('phase4a_record_duration'), value_type=float),
        }],
        condition=IfCondition(PythonExpression([
            "'", enable_phase4a, "' == 'true' and '",
            phase4a_run_trace, "' == 'true'"])),
        output='screen',
    )

    # Task 4 Phase 4A: the explicit, finite test command publisher. It is NOT a
    # walking producer; it publishes a bounded number of complete 24-dim
    # position targets and then exits. Started only with enable_phase4a:=true.
    # phase4a_run_probe:=false loads the actuator but leaves command publishing
    # to an external tool (used by the negative tests).
    phase4a_probe = Node(
        package='nao_gz_sim',
        executable='phase4a_probe.py',
        parameters=[{
            'use_sim_time': True,
            'command_topic': '/joint_command',
            'state_topic': '/joint_states',
            'joint_name': LaunchConfiguration('phase4a_joint'),
            'target_position_rad': ParameterValue(
                LaunchConfiguration('phase4a_target_rad'), value_type=float),
            'publish_count': ParameterValue(
                LaunchConfiguration('phase4a_publish_count'), value_type=int),
            'publish_period_sec': ParameterValue(
                LaunchConfiguration('phase4a_publish_period'), value_type=float),
        }],
        condition=IfCondition(PythonExpression([
            "'", enable_phase4a, "' == 'true' and '",
            enable_phase4b, "' == 'false' and '",
            LaunchConfiguration('phase4a_run_probe'), "' == 'true'"])),
        output='screen',
    )

    # Task 4 Phase 4B: read-only trace for the gait producer. Records clock,
    # command, joint state, foot contact, body pose and IMU and writes the raw
    # JSONL traces plus the phase4b summary. Publishes nothing.
    phase4b_evidence_dir = LaunchConfiguration('phase4b_evidence_dir')
    phase4b_trace = Node(
        package='nao_gz_sim',
        executable='phase4b_trace.py',
        parameters=[{
            'use_sim_time': True,
            'output_dir': phase4b_evidence_dir,
            'gait_method': LaunchConfiguration('phase4b_gait_method'),
            'record_duration_sec': ParameterValue(
                LaunchConfiguration('phase4b_record_duration'), value_type=float),
        }],
        condition=IfCondition(PythonExpression([
            "'", enable_phase4b, "' == 'true' and '",
            LaunchConfiguration('phase4b_run_gait'), "' == 'true'"])),
        output='screen',
    )

    # Task 4 Phase 4B: the minimal periodic gait command producer.
    phase4b_producer = Node(
        package='nao_gz_sim',
        executable='phase4b_producer.py',
        parameters=[{
            'use_sim_time': True,
            'command_topic': '/joint_command',
            'state_topic': '/joint_states',
            'gait_method': LaunchConfiguration('phase4b_gait_method'),
            'step_length_m': ParameterValue(
                LaunchConfiguration('phase4b_step_length_m'), value_type=float),
            'step_frequency_hz': ParameterValue(
                LaunchConfiguration('phase4b_step_frequency_hz'), value_type=float),
            'swing_height_m': ParameterValue(
                LaunchConfiguration('phase4b_swing_height_m'), value_type=float),
            'double_support_ratio': ParameterValue(
                LaunchConfiguration('phase4b_double_support_ratio'), value_type=float),
            'leg_extension_m': ParameterValue(
                LaunchConfiguration('phase4b_leg_extension_m'), value_type=float),
            'gait_blend_sec': ParameterValue(
                LaunchConfiguration('phase4b_gait_blend_sec'), value_type=float),
            'sway_amplitude_rad': ParameterValue(
                LaunchConfiguration('phase4b_sway_amplitude_rad'), value_type=float),
            'sway_sign': ParameterValue(
                LaunchConfiguration('phase4b_sway_sign'), value_type=float),
            'gait_phase_mode': LaunchConfiguration('phase4b_gait_phase_mode'),
            'arm_swing_rad': ParameterValue(
                LaunchConfiguration('phase4b_arm_swing_rad'), value_type=float),
            'arm_neutral_pitch_rad': ParameterValue(
                LaunchConfiguration('phase4b_arm_neutral_pitch_rad'), value_type=float),
            'arm_neutral_roll_rad': ParameterValue(
                LaunchConfiguration('phase4b_arm_neutral_roll_rad'), value_type=float),
            'elbow_bend_rad': ParameterValue(
                LaunchConfiguration('phase4b_elbow_bend_rad'), value_type=float),
            'arm_phase_offset_rad': ParameterValue(
                LaunchConfiguration('phase4b_arm_phase_offset_rad'), value_type=float),
            'arm_swing_sign': ParameterValue(
                LaunchConfiguration('phase4b_arm_swing_sign'), value_type=int),
            'foot_pitch_swing_rad': ParameterValue(
                LaunchConfiguration('phase4b_foot_pitch_swing_rad'),
                value_type=float),
            'command_rate_hz': ParameterValue(
                LaunchConfiguration('phase4b_command_rate_hz'), value_type=float),
            'duration_sec': ParameterValue(
                LaunchConfiguration('phase4b_duration_sec'), value_type=float),
            'start_delay_sec': ParameterValue(
                LaunchConfiguration('phase4b_start_delay_sec'), value_type=float),
            'stop_ramp_sec': ParameterValue(
                LaunchConfiguration('phase4b_stop_ramp_sec'), value_type=float),
            'hold_final_sec': ParameterValue(
                LaunchConfiguration('phase4b_hold_final_sec'), value_type=float),
            'stand_sec': ParameterValue(
                LaunchConfiguration('phase4b_stand_sec'), value_type=float),
            'walking_direction': LaunchConfiguration('phase4b_walking_direction'),
        }],
        condition=IfCondition(PythonExpression([
            "'", enable_phase4b, "' == 'true' and '",
            LaunchConfiguration('phase4b_run_gait'), "' == 'true'"])),
        output='screen',
    )

    # Task 4 Phase 4D: schema v1.0.0 trajectory collector. Read-only: it only
    # subscribes to the frozen sources and writes manifest.json + samples.jsonl.
    # It never publishes, never controls the robot and never touches the model.
    phase4d_collector = Node(
        package='nao_gz_sim',
        executable='phase4d_collector.py',
        parameters=[{
            'use_sim_time': True,
            'output_dir': LaunchConfiguration('phase4d_output_dir'),
            'summary_path': LaunchConfiguration('phase4d_summary_path'),
            'duration_sec': ParameterValue(
                LaunchConfiguration('phase4d_duration_sec'), value_type=float),
            'max_samples': ParameterValue(
                LaunchConfiguration('phase4d_max_samples'), value_type=int),
            'overwrite': ParameterValue(
                LaunchConfiguration('phase4d_overwrite'), value_type=bool),
            'watermark_margin_sec': ParameterValue(
                LaunchConfiguration('phase4d_watermark_margin_sec'), value_type=float),
            'validator_script': LaunchConfiguration('phase4d_validator_script'),
        }],
        condition=IfCondition(PythonExpression([
            "'", enable_phase4d, "' == 'true' and '",
            LaunchConfiguration('phase4d_run_collector'), "' == 'true'"])),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Run Gazebo Sim with the GUI (false = server only).'),
        DeclareLaunchArgument(
            'gz_seed', default_value='',
            description='Optional physics RNG seed forwarded to gz sim --seed '
                        '(empty = historical command, no --seed).'),
        DeclareLaunchArgument(
            'world_name', default_value='nao_world',
            description='Name of the world in nao_world.sdf.'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.35',
            description='Initial z of the spawned NAO model (feet clear ground).'),
        DeclareLaunchArgument(
            'spawn_delay', default_value='3.0',
            description='Seconds to wait for the world before spawning.'),
        DeclareLaunchArgument(
            'use_rsp', default_value='true',
            description='Start robot_state_publisher with nao_simple.urdf.'),
        DeclareLaunchArgument(
            'physics_engine', default_value='gz-physics-dartsim-plugin',
            description='Gazebo Physics engine plugin for gz sim 8.'),
        DeclareLaunchArgument(
            'enable_contact_converter', default_value='true',
            description='Start the Task 3E contact bridge/converter (false = negative test).'),
        DeclareLaunchArgument(
            'enable_phase4a', default_value='false',
            description='Load the Phase 4A actuator and start the finite probe + trace.'),
        DeclareLaunchArgument(
            'phase4a_run_probe', default_value='true',
            description='Start the finite Phase 4A probe (false = actuator only).'),
        DeclareLaunchArgument(
            'phase4a_run_trace', default_value='true',
            description='Start the Phase 4A trace recorder.'),
        DeclareLaunchArgument(
            'phase4a_joint', default_value='HeadYaw',
            description='Joint exercised by the Phase 4A probe.'),
        DeclareLaunchArgument(
            'phase4a_target_rad', default_value='0.3',
            description='Safe position target (rad) for the Phase 4A probe joint.'),
        DeclareLaunchArgument(
            'phase4a_publish_count', default_value='10',
            description='Finite number of Phase 4A commands to publish.'),
        DeclareLaunchArgument(
            'phase4a_publish_period', default_value='0.2',
            description='Phase 4A probe publish period in simulation seconds.'),
        DeclareLaunchArgument(
            'phase4a_record_duration', default_value='12.0',
            description='Phase 4A trace duration in simulation seconds after first state.'),
        DeclareLaunchArgument(
            'phase4a_evidence_dir', default_value=os.path.join(
                os.getcwd(), 'docs', 'task4_phase4a_evidence'),
            description='Directory for the Phase 4A trace and summary outputs.'),
        DeclareLaunchArgument(
            'enable_phase4b', default_value='false',
            description='Load the Phase 4B gait producer and its read-only trace.'),
        DeclareLaunchArgument(
            'actuator_p_gain', default_value='8.0',
            description='P gain of the runtime actuator position servo.'),
        DeclareLaunchArgument(
            'actuator_max_velocity', default_value='2.0',
            description='Max joint velocity command of the runtime actuator [rad/s].'),
        DeclareLaunchArgument(
            'actuator_hold_on_start', default_value='false',
            description='Hold the start joint pose until the first command (Phase 4B safety).'),
        DeclareLaunchArgument(
            'phase4b_run_gait', default_value='false',
            description='Actually run the Phase 4B producer/trace (false = actuator only).'),
        DeclareLaunchArgument(
            'phase4b_gait_method', default_value='walk',
            description='Phase 4B gait method: stand | walk.'),
        DeclareLaunchArgument(
            'phase4b_step_length_m', default_value='0.03',
            description='Phase 4B forward foot step length [m].'),
        DeclareLaunchArgument(
            'phase4b_step_frequency_hz', default_value='0.8',
            description='Phase 4B gait cycle frequency [Hz].'),
        DeclareLaunchArgument(
            'phase4b_swing_height_m', default_value='0.015',
            description='Phase 4B swing foot clearance [m].'),
        DeclareLaunchArgument(
            'phase4b_leg_extension_m', default_value='0.24401',
            description='Phase 4B hip-to-sole leg extension [m] (bent-knee stance).'),
        DeclareLaunchArgument(
            'phase4b_gait_blend_sec', default_value='1.5',
            description='Phase 4B stance->gait smooth blend [simulation s].'),
        DeclareLaunchArgument(
            'phase4b_double_support_ratio', default_value='0.2',
            description='Phase 4B double-support fraction of the cycle.'),
        DeclareLaunchArgument(
            'phase4b_sway_amplitude_rad', default_value='0.15',
            description='Phase 4B lateral hip/ankle roll weight-shift amplitude [rad].'),
        DeclareLaunchArgument(
            'phase4b_sway_sign', default_value='1.0',
            description='Phase 4B weight-shift sign (+1 historical, -1 pelvis over '
                        'the stance foot).'),
        DeclareLaunchArgument(
            'phase4b_gait_phase_mode', default_value='legacy',
            description='Phase 4B phase scheduling: legacy | explicit_support_swing.'),
        DeclareLaunchArgument(
            'phase4b_arm_swing_rad', default_value='0.2',
            description='Phase 4B shoulder pitch swing amplitude [rad].'),
        DeclareLaunchArgument(
            'phase4b_arm_neutral_pitch_rad', default_value='1.5',
            description='Phase 4E natural hanging-arm shoulder pitch baseline [rad].'),
        DeclareLaunchArgument(
            'phase4b_arm_neutral_roll_rad', default_value='0.0',
            description='Phase 4E natural shoulder roll (mirrored L/R) [rad].'),
        DeclareLaunchArgument(
            'phase4b_elbow_bend_rad', default_value='0.3',
            description='Phase 4E natural elbow bend (mirrored L/R) [rad].'),
        DeclareLaunchArgument(
            'phase4b_arm_phase_offset_rad', default_value='0.0',
            description='Phase 4E arm swing phase offset [rad].'),
        DeclareLaunchArgument(
            'phase4b_arm_swing_sign', default_value='1',
            description='Phase 4E arm swing sign (+1 natural, -1 legacy zero-pose CONTROL).'),
        DeclareLaunchArgument(
            'phase4b_foot_pitch_swing_rad', default_value='0.10',
            description='Phase 4B swing-foot pitch amplitude [rad].'),
        DeclareLaunchArgument(
            'phase4b_command_rate_hz', default_value='100.0',
            description='Phase 4B command publication rate [Hz].'),
        DeclareLaunchArgument(
            'phase4b_duration_sec', default_value='12.0',
            description='Phase 4B gait duration [simulation s].'),
        DeclareLaunchArgument(
            'phase4b_start_delay_sec', default_value='0.5',
            description='Phase 4B delay after the model is ready before gait starts [s].'),
        DeclareLaunchArgument(
            'phase4b_stop_ramp_sec', default_value='1.5',
            description='Phase 4B ramp back to stance at the end [simulation s].'),
        DeclareLaunchArgument(
            'phase4b_hold_final_sec', default_value='2.0',
            description='Phase 4B final stance hold after the stop ramp [simulation s].'),
        DeclareLaunchArgument(
            'phase4b_stand_sec', default_value='12.0',
            description='Phase 4B stand-baseline duration [simulation s].'),
        DeclareLaunchArgument(
            'phase4b_walking_direction', default_value='forward',
            description='Phase 4B walking direction: forward | backward.'),
        DeclareLaunchArgument(
            'phase4b_record_duration', default_value='0.0',
            description='Phase 4B trace duration [sim s] after first state (0 = until shutdown).'),
        DeclareLaunchArgument(
            'phase4b_evidence_dir', default_value=os.path.join(
                os.getcwd(), 'docs', 'task4_phase4b_evidence'),
            description='Directory for the Phase 4B traces and summary.'),
        DeclareLaunchArgument(
            'enable_phase4d', default_value='false',
            description='Start the Phase 4D trajectory collector (read-only).'),
        DeclareLaunchArgument(
            'phase4d_run_collector', default_value='false',
            description='Actually run the Phase 4D collector (false = nothing).'),
        DeclareLaunchArgument(
            'phase4d_output_dir', default_value=os.path.join(
                os.getcwd(), 'docs', 'task4_phase4d_evidence', 'positive_collection'),
            description='Output dataset directory (manifest.json + samples.jsonl).'),
        DeclareLaunchArgument(
            'phase4d_summary_path', default_value=os.path.join(
                os.getcwd(), 'docs', 'task4_phase4d_evidence', 'collector_summary.json'),
            description='Collector statistics output (must be outside output_dir).'),
        DeclareLaunchArgument(
            'phase4d_duration_sec', default_value='8.0',
            description='Phase 4D collection duration in simulation seconds.'),
        DeclareLaunchArgument(
            'phase4d_max_samples', default_value='0',
            description='Optional hard sample cap (0 = disabled).'),
        DeclareLaunchArgument(
            'phase4d_overwrite', default_value='false',
            description='Allow replacing an existing Phase 4D dataset directory.'),
        DeclareLaunchArgument(
            'phase4d_watermark_margin_sec', default_value='0.02',
            description='Extra simulation seconds before finalising a pending sample.'),
        DeclareLaunchArgument(
            'phase4d_validator_script', default_value=os.path.join(
                os.getcwd(), 'tools', 'validate_task4_phase4d_dataset.py'),
            description='Independent golden validator run on staging before publish '
                        '(empty = skip).'),
        OpaqueFunction(function=_gz_sim_actions),
        OpaqueFunction(function=_phase4_model_spawn),
        clock_bridge,
        joint_state_bridge,
        joint_state_converter,
        body_pose_converter,
        imu_bridge,
        imu_converter,
        contact_bridge,
        contact_converter,
        world_tf_converter,
        robot_state_publisher,
        phase4a_trace,
        phase4a_probe,
        phase4b_trace,
        phase4b_producer,
        phase4d_collector,
    ])
