#!/usr/bin/env python
import argparse
import math
import numpy as np
import os
from skrobot.coordinates.math import rotation_matrix
from yamaopt.solver import KinematicSolver, SolverConfig
from yamaopt.polygon_constraint import polygon_to_trans_constraint
from yamaopt.visualizer import VisManager

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-robot', type=str, default='pr2', help='robot name')
    parser.add_argument(
        '--config_path', type=str, default='', help='path to robot config file.')
    parser.add_argument('-hover', type=float, default='0.05', help='hover distance')
    parser.add_argument('-margin', type=float, default='5.0', help='joint limit margin [deg]')
    parser.add_argument('--visualize', action='store_true', help='visualize')
    parser.add_argument('--use_base', action='store_true', help='with base')
    parser.add_argument('--limit_base', action='store_true', help='limit movable area of base')
    args = parser.parse_args()
    robot_name = args.robot
    config_path = args.config_path
    visualize = args.visualize
    use_base = args.use_base
    d_hover = args.hover
    joint_limit_margin = args.margin
    limit_base = args.limit_base

    if robot_name == 'fetch':
        config_path = "../config/fetch_conf.yaml"
    elif robot_name == 'pr2':
        config_path = "../config/pr2_conf.yaml"
    else:
        raise Exception

    config = SolverConfig.from_config_path(config_path, use_base=use_base)
    kinsol = KinematicSolver(config)

    polygon1 = np.array([[1.0, -0.5, -0.5], [1.0, 0.5, -0.5], [1.0, 0.5, 0.5], [1.0, -0.5, 0.5]]) + np.array([0, 0, 1.0])
    polygon2 = polygon1.dot(rotation_matrix(math.pi / 2.0, [0, 0, 1.0]).T)
    polygon3 = polygon1.dot(rotation_matrix(-math.pi / 2.0, [0, 0, 1.0]).T)
    polygon4 = np.array([[1.0, 0.2, 0.2], [1.0, 0.5, -0.5], [1.0, 0.5, 0.5], [1.0, -0.5, 0.5]]) + np.array([0, 0, 1.0])
    polygons = [polygon1, polygon2, polygon3, polygon4]

    target_obj_pos = np.array([-0.1, 0.7, 0.3])

    if use_base and limit_base:
        movable_polygon = np.array(
            [[0.5, 0, 0], [0, 0.5, 0], [-0.5, 0, 0], [0, -0.5, 0]])
        movable_polygon += np.array([-1.4, 0.5, 0.0])
    else:
        movable_polygon = None

    q_init = -np.ones(len(kinsol.control_joint_ids)) * 0.4
    sol = kinsol.solve_multiple(q_init, polygons, target_obj_pos,
                                movable_polygon=movable_polygon,
                                d_hover=d_hover,
                                joint_limit_margin=joint_limit_margin)
    if sol.success:
        print('optimization succeeded.')
    else:
        print('optimization failed.')
        assert sol.success

    if visualize:
        # visualize
        vm = VisManager(config)
        vm.add_target(target_obj_pos)
        vm.reflect_solver_result(
            sol, polygons, movable_polygon=movable_polygon,
            show_polygon_axis=True)
        vm.show_while()
