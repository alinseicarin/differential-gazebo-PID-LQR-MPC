#!/usr/bin/env python3
"""Fail fast when the benchmark's duplicated physical/protocol values drift."""

import ast
import csv
import importlib.util
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_ROOT = ROOT / 'src' / 'my_robot_controller'
DESCRIPTION_ROOT = ROOT / 'src' / 'my_robot_description'

CONTROLLER_CONFIGS = {
    'pid': ('pid_node', CONTROLLER_ROOT / 'config' / 'pid_cascade.yaml'),
    'lqr': ('lqr_node', CONTROLLER_ROOT / 'config' / 'lqr.yaml'),
    'mpc': ('mpc_node', CONTROLLER_ROOT / 'config' / 'mpc.yaml'),
}
COMMON_CONTROLLER_PARAMETERS = (
    'nominal_control_frequency', 'max_control_dt', 'goal_tolerance',
    'goal_heading_tolerance', 'max_linear_velocity',
    'max_angular_velocity', 'translation_stop_lateral_error',
    'translation_stop_heading_error', 'odom_timeout',
    'startup_settling_time',
)
COMMON_REFERENCE_PARAMETERS = (
    'search_window', 'reference_linear_velocity', 'curvature_speed_gain',
    'endpoint_slowdown_distance', 'maximum_reference_curvature',
    'trajectory_spatial_step', 'maximum_reference_linear_acceleration',
    'maximum_reference_linear_deceleration',
    'maximum_reference_angular_velocity',
)


class Audit:
    def __init__(self):
        self.checks = 0
        self.issues = []

    def require(self, condition, message):
        self.checks += 1
        if not condition:
            self.issues.append(message)

    def equal(self, actual, expected, message):
        self.require(actual == expected, f'{message}: {actual!r} != {expected!r}')

    def close(self, actual, expected, message, tolerance=1.0e-9):
        self.require(
            math.isclose(actual, expected, rel_tol=tolerance, abs_tol=tolerance),
            f'{message}: {actual:.12g} != {expected:.12g}',
        )


def yaml_parameters(path, node_name):
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    return document[node_name]['ros__parameters']


def declared_defaults(path):
    source = path.read_text(encoding='utf-8')
    pattern = re.compile(
        r'declare_parameter<[^>]+>\("([^"]+)",\s*([^;]+)\);'
    )
    values = {}
    for name, expression in pattern.findall(source):
        normalized = expression.strip()
        if normalized == 'true':
            values[name] = True
            continue
        if normalized == 'false':
            values[name] = False
            continue
        try:
            values[name] = ast.literal_eval(normalized)
        except (SyntaxError, ValueError):
            pass
    return values


def xyz(element):
    return tuple(float(value) for value in element.attrib['xyz'].split())


def audit_configuration(audit):
    configs = {
        family: yaml_parameters(path, node)
        for family, (node, path) in CONTROLLER_CONFIGS.items()
    }
    reference = yaml.safe_load(
        (CONTROLLER_ROOT / 'config' / 'trajectory_reference.yaml').read_text(
            encoding='utf-8'
        )
    )['/**']['ros__parameters']

    for parameter in COMMON_CONTROLLER_PARAMETERS:
        values = {family: config.get(parameter) for family, config in configs.items()}
        audit.require(
            len(set(values.values())) == 1 and None not in values.values(),
            f'common controller parameter differs: {parameter}={values}',
        )
    for parameter in COMMON_REFERENCE_PARAMETERS:
        expected = reference.get(parameter)
        audit.require(expected is not None, f'reference parameter is missing: {parameter}')
        for family, config in configs.items():
            audit.equal(
                config.get(parameter), expected,
                f'{family} reference parameter {parameter}',
            )

    for family, (node, path) in CONTROLLER_CONFIGS.items():
        config = configs[family]
        defaults = declared_defaults(CONTROLLER_ROOT / 'src' / f'{family}_node.cpp')
        for parameter, configured in config.items():
            if parameter == 'use_sim_time':
                continue
            audit.require(
                parameter in defaults,
                f'{family} YAML parameter is not declared: {parameter}',
            )
            if parameter in defaults:
                audit.equal(
                    defaults[parameter], configured,
                    f'{family} fallback default for {parameter}',
                )

    max_reference_yaw = reference['maximum_reference_angular_velocity']
    max_actuator_yaw = configs['pid']['max_angular_velocity']
    audit.require(
        max_reference_yaw <= max_actuator_yaw,
        'reference yaw-rate limit exceeds actuator yaw-rate limit',
    )
    return configs, reference


def audit_robot_model(audit, configs, reference):
    robot = ET.parse(DESCRIPTION_ROOT / 'urdf' / 'my_robot.urdf').getroot()
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}
    links = {link.attrib['name']: link for link in robot.findall('link')}

    left_origin = xyz(joints['left_wheel_joint'].find('origin'))
    right_origin = xyz(joints['right_wheel_joint'].find('origin'))
    separation = abs(left_origin[1] - right_origin[1])
    wheel = links['left_wheel'].find('collision/geometry/cylinder')
    wheel_radius = float(wheel.attrib['radius'])
    wheel_width = float(wheel.attrib['length'])
    audit.require(wheel_radius > 0.0 and wheel_width > 0.0, 'wheel dimensions are not positive')

    plugins = {
        plugin.attrib['name']: plugin
        for gazebo in robot.findall('gazebo')
        for plugin in gazebo.findall('plugin')
    }
    drive = plugins['diff_drive']
    audit.close(float(drive.findtext('wheel_separation')), separation,
                'plugin/URDF wheel separation')
    audit.close(float(drive.findtext('wheel_diameter')), 2.0 * wheel_radius,
                'plugin/URDF wheel diameter')

    joint_limits = []
    for side in ('left', 'right'):
        joint = joints[f'{side}_wheel_joint']
        cylinder = links[f'{side}_wheel'].find('collision/geometry/cylinder')
        visual = links[f'{side}_wheel'].find('visual/geometry/cylinder')
        audit.close(float(cylinder.attrib['radius']), wheel_radius,
                    f'{side} collision radius')
        audit.close(float(visual.attrib['radius']), wheel_radius,
                    f'{side} visual radius')
        audit.close(float(cylinder.attrib['length']), wheel_width,
                    f'{side} collision width')
        audit.close(float(visual.attrib['length']), wheel_width,
                    f'{side} visual width')
        limit = joint.find('limit')
        audit.close(float(limit.attrib['effort']), float(drive.findtext('max_wheel_torque')),
                    f'{side} effort/plugin torque')
        joint_limits.append(float(limit.attrib['velocity']))

    maximum_linear = configs['pid']['max_linear_velocity']
    maximum_angular = configs['pid']['max_angular_velocity']
    required_wheel_rate = (
        maximum_linear + 0.5 * separation * maximum_angular
    ) / wheel_radius
    audit.require(
        min(joint_limits) + 1.0e-9 >= required_wheel_rate,
        f'wheel joint limit {min(joint_limits):.6g} rad/s is below '
        f'{required_wheel_rate:.6g} rad/s required by the body-command box',
    )

    chassis_box = links['chassis'].find('collision/geometry/box').attrib['size']
    chassis_visual = links['chassis'].find('visual/geometry/box').attrib['size']
    audit.equal(chassis_visual, chassis_box, 'chassis visual/collision dimensions')
    caster_collision = links['caster_wheel'].find('collision/geometry/sphere')
    caster_visual = links['caster_wheel'].find('visual/geometry/sphere')
    caster_radius = float(caster_collision.attrib['radius'])
    audit.equal(caster_visual.attrib['radius'], caster_collision.attrib['radius'],
                'caster visual/collision radius')

    # Check the declared ideal solid-body inertias against their dimensions.
    chassis_dimensions = tuple(float(value) for value in chassis_box.split())
    chassis_inertial = links['chassis'].find('inertial')
    chassis_mass = float(chassis_inertial.find('mass').attrib['value'])
    chassis_inertia = chassis_inertial.find('inertia').attrib
    length, width, height = chassis_dimensions
    audit.close(float(chassis_inertia['ixx']), chassis_mass * (width**2 + height**2) / 12.0,
                'chassis Ixx', tolerance=1.0e-6)
    audit.close(float(chassis_inertia['iyy']), chassis_mass * (length**2 + height**2) / 12.0,
                'chassis Iyy', tolerance=1.0e-6)
    audit.close(float(chassis_inertia['izz']), chassis_mass * (length**2 + width**2) / 12.0,
                'chassis Izz', tolerance=1.0e-6)

    for side in ('left', 'right'):
        inertial = links[f'{side}_wheel'].find('inertial')
        mass = float(inertial.find('mass').attrib['value'])
        inertia = inertial.find('inertia').attrib
        transverse = mass * (3.0 * wheel_radius**2 + wheel_width**2) / 12.0
        axial = 0.5 * mass * wheel_radius**2
        audit.close(float(inertia['ixx']), transverse, f'{side} wheel Ixx', tolerance=1.0e-6)
        audit.close(float(inertia['iyy']), transverse, f'{side} wheel Iyy', tolerance=1.0e-6)
        audit.close(float(inertia['izz']), axial, f'{side} wheel Izz', tolerance=1.0e-6)

    caster_inertial = links['caster_wheel'].find('inertial')
    caster_mass = float(caster_inertial.find('mass').attrib['value'])
    caster_inertia = caster_inertial.find('inertia').attrib
    sphere_inertia = 0.4 * caster_mass * caster_radius**2
    for axis in ('ixx', 'iyy', 'izz'):
        audit.close(float(caster_inertia[axis]), sphere_inertia,
                    f'caster {axis}', tolerance=1.0e-6)

    chassis_height = xyz(joints['base_joint'].find('origin'))[2]
    wheel_ground_z = chassis_height + left_origin[2] - wheel_radius
    caster_ground_z = (
        chassis_height + xyz(joints['caster_wheel_joint'].find('origin'))[2]
        - caster_radius
    )
    audit.close(wheel_ground_z, 0.0, 'drive-wheel ground contact height')
    audit.close(caster_ground_z, 0.0, 'caster ground contact height')

    nominal_frequency = configs['pid']['nominal_control_frequency']
    audit.close(float(drive.findtext('update_rate')), nominal_frequency,
                'diff-drive/controller frequency')
    audit.close(float(plugins['ground_truth'].findtext('update_rate')), nominal_frequency,
                'ground-truth/controller frequency')
    audit.close(float(plugins['joint_state_publisher'].findtext('update_rate')),
                nominal_frequency, 'joint-state/controller frequency')
    ekf = yaml_parameters(DESCRIPTION_ROOT / 'config' / 'ekf.yaml', 'ekf_filter_node')
    audit.close(float(ekf['frequency']), nominal_frequency, 'EKF/controller frequency')

    imu = robot.find("gazebo[@reference='imu_link']/sensor")
    audit.require(float(imu.findtext('update_rate')) >= nominal_frequency,
                  'IMU update rate is lower than EKF/controller rate')
    audit.require(float(drive.findtext('max_wheel_acceleration')) >=
                  reference['maximum_reference_linear_acceleration'],
                  'drive-wheel acceleration is below reference linear acceleration')

    physics_signatures = []
    for world_path in sorted((DESCRIPTION_ROOT / 'worlds').glob('*.world')):
        world = ET.parse(world_path).getroot().find('world')
        physics = world.find('physics')
        signature = (
            physics.findtext('max_step_size'),
            physics.findtext('real_time_factor'),
            physics.findtext('real_time_update_rate'),
            physics.findtext('ode/solver/type'),
            physics.findtext('ode/solver/iters'),
            physics.findtext('ode/solver/sor'),
        )
        physics_signatures.append((world_path.name, signature))
    audit.require(
        len({signature for _, signature in physics_signatures}) == 1,
        f'world physics settings differ: {physics_signatures}',
    )
    return separation


def audit_tracks(audit):
    generator_path = ROOT / 'generate_tracks.py'
    spec = importlib.util.spec_from_file_location('benchmark_tracks', generator_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    expected_tracks = (
        module.track_1, module.track_2, module.track_3, module.track_4,
        module.track_5, module.track_6, module.track_7,
    )
    for index, expected in enumerate(expected_tracks, start=1):
        paths = list(ROOT.glob(f'track_{index}_*.csv'))
        audit.equal(len(paths), 1, f'number of CSV files for track {index}')
        if len(paths) != 1:
            continue
        with paths[0].open(newline='', encoding='utf-8') as stream:
            actual = [tuple(float(value) for value in row) for row in csv.reader(stream)]
        quantized = [module.quantize_waypoint(point) for point in expected]
        audit.equal(actual, quantized, f'{paths[0].name} differs from generator')
        audit.require(
            all(math.dist(a, b) > 1.0e-9 for a, b in zip(actual, actual[1:])),
            f'{paths[0].name} contains a zero-length segment',
        )
    for name, points in (
            ('curve', module.track_2), ('circle', module.track_4),
            ('figure eight', module.track_5)):
        try:
            module.validate_smooth_track(name, points)
        except ValueError as error:
            audit.require(False, str(error))
        else:
            audit.require(True, '')


def launch_shape(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    arguments = set()
    nodes = {}
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        if isinstance(call.func, ast.Name) and call.func.id == 'DeclareLaunchArgument':
            if call.args and isinstance(call.args[0], ast.Constant):
                arguments.add(call.args[0].value)
        if not (isinstance(call.func, ast.Name) and call.func.id == 'Node'):
            continue
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        executable = keywords.get('executable')
        if not isinstance(executable, ast.Constant):
            continue
        parameter_keys = set()
        for item in ast.walk(keywords.get('parameters', ast.List(elts=[]))):
            if isinstance(item, ast.Dict):
                parameter_keys.update(
                    key.value for key in item.keys
                    if isinstance(key, ast.Constant) and isinstance(key.value, str)
                )
        nodes[executable.value] = parameter_keys
    return arguments, nodes


def audit_launch_parity(audit, configs, wheel_separation):
    shapes = {
        family: launch_shape(
            CONTROLLER_ROOT / 'launch' / f'{family}_perturbation_suite.launch.py'
        )
        for family in ('pid', 'lqr', 'mpc')
    }
    pid_arguments = shapes['pid'][0]
    audit.equal(shapes['lqr'][0], pid_arguments, 'PID/LQR perturbation arguments')
    audit.equal(
        shapes['mpc'][0] - {'prediction_horizon_steps'}, pid_arguments,
        'PID/MPC perturbation arguments except MPC horizon',
    )
    for executable in (
            'command_disturbance_injector', 'odometry_disturbance_injector',
            'trajectory_evaluator_node'):
        keys = {
            family: shape[1].get(executable) for family, shape in shapes.items()
        }
        audit.require(
            len({frozenset(value or set()) for value in keys.values()}) == 1 and
            None not in keys.values(),
            f'{executable} parameter interface differs: {keys}',
        )

    expected_literals = {
        'maximum_abs_linear_velocity': configs['pid']['max_linear_velocity'],
        'maximum_abs_angular_velocity': configs['pid']['max_angular_velocity'],
    }
    for family in ('pid', 'lqr', 'mpc'):
        source = (
            CONTROLLER_ROOT / 'launch' /
            f'{family}_perturbation_suite.launch.py'
        ).read_text(encoding='utf-8')
        separation_match = re.search(
            r"DeclareLaunchArgument\('wheel_separation',\s*default_value='([^']+)'\)",
            source,
        )
        audit.require(
            separation_match is not None,
            f'{family} perturbation launch lacks wheel-separation default',
        )
        if separation_match is not None:
            audit.close(float(separation_match.group(1)), wheel_separation,
                        f'{family} launch wheel separation')
        for parameter, expected in expected_literals.items():
            match = re.search(rf"'{parameter}':\s*([0-9.eE+-]+)", source)
            audit.require(
                match is not None,
                f'{family} perturbation launch lacks {parameter}',
            )
            if match is not None:
                audit.close(float(match.group(1)), expected,
                            f'{family} launch {parameter}')

    runner = (ROOT / 'scripts' / 'run_pid_perturbation_case.sh').read_text(
        encoding='utf-8'
    )
    runner_separation = re.search(r'wheel_separation:=([0-9.eE+-]+)', runner)
    audit.require(
        runner_separation is not None,
        'perturbation runner lacks explicit wheel separation',
    )
    if runner_separation is not None:
        audit.close(float(runner_separation.group(1)), wheel_separation,
                    'perturbation runner wheel separation')


def main():
    audit = Audit()
    try:
        configs, reference = audit_configuration(audit)
        wheel_separation = audit_robot_model(audit, configs, reference)
        audit_tracks(audit)
        audit_launch_parity(audit, configs, wheel_separation)
    except Exception as error:  # Convert parser failures into a clear preflight failure.
        audit.issues.append(f'audit could not complete: {error}')

    print('PROJECT CONSISTENCY AUDIT')
    print(f'checks={audit.checks}')
    print(f'issues={len(audit.issues)}')
    if audit.issues:
        for issue in audit.issues:
            print(f'FAIL: {issue}')
        return 1
    print('PASS: controller, reference, URDF/Gazebo, track, and launch invariants agree.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
