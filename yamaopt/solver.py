import os
import math
import copy
import attr
from tinyfk import RobotModel
import yaml
import numpy as np
import scipy.optimize
import skrobot
from skrobot.model.joint import RotationalJoint
from skrobot.model.joint import FixedJoint
from skrobot.model.joint import LinearJoint
from skrobot.model.joint import OmniWheelJoint

from yamaopt.polygon_constraint import polygon_to_trans_constraint
from yamaopt.polygon_constraint import polygon_to_desired_rpy
from yamaopt.polygon_constraint import ConcavePolygonException
from yamaopt.utils import scipinize

@attr.s # like a dataclass in python3
class SolverConfig(object):
    use_base = attr.ib()
    urdf_path = attr.ib()
    optimization_frame = attr.ib()
    control_joint_names = attr.ib()
    endeffector_link_name = attr.ib()

    @classmethod
    def from_config_path(cls, 
            config_path, 
            use_base=False, 
            joint_limit_margin=None # [degree] or None
            ):
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)

        return cls(
                use_base,
                urdf_path = cfg['urdf_path'],
                optimization_frame = cfg['optimization_frame'],
                control_joint_names = cfg['control_joint_names'],
                endeffector_link_name = cfg['endeffector_link_name'],
                )

@attr.s # like a dataclass in python3
class SolverResult(object):
    success = attr.ib()
    x = attr.ib()
    fun = attr.ib()

    # additional infos
    end_coords = attr.ib()
    target_polygon = attr.ib()
    _d_hover = attr.ib()
    _sol_scipy = attr.ib()

class KinematicSolver:
    def __init__(self, config):
        urdf_path = os.path.expanduser(config.urdf_path)
        self.kin = RobotModel(urdf_path)

        robot_model = skrobot.model.RobotModel() # Here this model is used only for obtaining joint type (as an urdf parser)
        robot_model.load_urdf_file(urdf_path)

        self.config = config
        self.control_joint_ids = self.kin.get_joint_ids(config.control_joint_names)
        joint_limits = self.kin.get_joint_limits(self.control_joint_ids)
        joint_types = [type(robot_model.__dict__[jn]) for jn in config.control_joint_names]

        if self.config.use_base:
            joint_limits.extend([[None, None]] * 3) # for x, y, theta
            joint_types.extend([LinearJoint, LinearJoint, RotationalJoint])

        self.joint_limits = joint_limits
        self.joint_types = joint_types
        self.end_effector_id = self.kin.get_link_ids([config.endeffector_link_name])[0]

    @property
    def dof(self): return len(self.control_joint_ids) + 3 * (self.config.use_base)

    # TODO lru cache
    def forward_kinematics(self, q):
        assert isinstance(q, np.ndarray) and q.ndim == 1
        with_jacobian = True 
        use_rotation = True
        use_base = self.config.use_base
        
        link_ids = [self.end_effector_id]
        joint_ids = self.control_joint_ids
        P, J = self.kin.solve_forward_kinematics(
                [q], link_ids, joint_ids, use_rotation, use_base, with_jacobian)
        return P, J

    def create_objective_function(self, target_obs_pos):

        def f(q):
            P_whole, J_whole = self.forward_kinematics(q)
            P_pos = P_whole[:, :3]
            J_pos = J_whole[:3, :]
            val = np.sum((P_pos.flatten() - target_obs_pos) ** 2)
            grad = 2 * (P_pos.flatten() - target_obs_pos).dot(J_pos)
            return val, grad

        return f

    # TODO: We should seperate this function: for hand and for base?
    def configuration_constraint_from_polygon(
            self, np_polygon, movable_polygon=None, d_hover=0.0):
        lin_ineq, lin_eq = polygon_to_trans_constraint(np_polygon, d_hover)
        b_lin_ineq = polygon_to_trans_constraint(movable_polygon, d_hover=0.0)[0]
        rpy_desired = polygon_to_desired_rpy(np_polygon)
        # print(np_polygon)
        # print(movable_polygon)
        # print("lin_ineq")
        # print(lin_ineq)
        print("desired")
        print(rpy_desired)

        def hand_ineq_constraint(q):
            P_whole, J_whole = self.forward_kinematics(q)
            P_pos = P_whole[:, :3]
            J_pos = J_whole[:3, :]
            val = ((lin_ineq.A.dot(P_pos.T)).T - lin_ineq.b).flatten()
            jac = lin_ineq.A.dot(J_pos)
            # print(P_pos)
            # print(J_pos)
            # print('hand')
            # print(val)
            # print(jac)
            return val, jac

        def hand_eq_constraint(q):
            P_whole, J_whole = self.forward_kinematics(q)
            P_pos, P_rot = P_whole[:, :3], P_whole[:, 3:]
            J_pos, J_rot = J_whole[:3, :], J_whole[3:, :]
            val_pos = ((lin_eq.A.dot(P_pos.T)).T - lin_eq.b).flatten()
            jac_pos = lin_eq.A.dot(J_pos)

            val_rot = P_rot.flatten() - rpy_desired
            jac_rot = J_rot
            return np.hstack([val_pos, val_rot]), np.vstack([jac_pos, jac_rot])

        def base_ineq_constraint(q):
            P = np.array([q[-3:]])
            P[0][-1] = 0  # P = [x, y, z=0]
            # J = np.zeros(())
            # J = np.eye(len(q))
            # J[:-3] = 0
            # J[-1] = 0
            J = np.zeros((3, 10))  # 10 -> len(q)
            J[0][-3] = 1.0
            J[1][-2] = 1.0
            print(b_lin_ineq)
            val = ((b_lin_ineq.A.dot(P.T)).T - b_lin_ineq.b).flatten()
            jac = b_lin_ineq.A.dot(J)
            # print('base')
            # print(q)
            # print(P)
            # print(J)
            # print(val)
            # print(jac)
            return val, jac

        return hand_ineq_constraint, hand_eq_constraint, base_ineq_constraint

    def solve(self, q_init, np_polygon, target_obs_pos, movable_polygon=None,
              d_hover=0.0, joint_limit_margin=0.0):
        if self.config.use_base:
            q_init = np.hstack((q_init, np.zeros(3)))
        assert len(q_init) == self.dof

        # Constraint functions
        f_ineq, f_eq, b_ineq = self.configuration_constraint_from_polygon(
            np_polygon, movable_polygon=movable_polygon, d_hover=d_hover)
        eq_const_scipy, eq_const_jac_scipy = scipinize(f_eq)
        eq_dict = {'type': 'eq', 'fun': eq_const_scipy,
                   'jac': eq_const_jac_scipy}
        ineq_const_scipy, ineq_const_jac_scipy = scipinize(f_ineq)
        ineq_dict = {'type': 'ineq', 'fun': ineq_const_scipy,
                     'jac': ineq_const_jac_scipy}
        if movable_polygon is None:
            cons = [eq_dict, ineq_dict]
        else:
            b_ineq_const_scipy, b_ineq_const_jac_scipy = scipinize(b_ineq)
            b_ineq_dict = {'type': 'ineq', 'fun': b_ineq_const_scipy,
                           'jac': b_ineq_const_jac_scipy}
            cons = [eq_dict, ineq_dict, b_ineq_dict]

        # Objective function
        f_obj = self.create_objective_function(target_obs_pos)
        f, jac = scipinize(f_obj)

        # Bounds
        joint_limits_tight = copy.deepcopy(self.joint_limits)
        margin = joint_limit_margin * math.pi / 180.0
        for i in range(len(joint_limits_tight)):
            joint_type = self.joint_types[i]
            if joint_type == LinearJoint:
                continue
            is_infinite_rotational_joint = (None in joint_limits_tight[i])
            if not is_infinite_rotational_joint:
                joint_limits_tight[i][0] += margin # tighten lower bound
                joint_limits_tight[i][1] -= margin # tighten upper bound

        # Solve optimization
        sol = scipy.optimize.minimize(
            f, q_init, method='SLSQP', jac=jac,
            constraints=cons, bounds=joint_limits_tight)

        """
        if output_gif:
            urdf_path = os.path.expanduser(self.config.urdf_path)
            vis = PybulletVisualizer(urdf_path, self.config.control_joint_names, False)
            n_seq = 20
            dq = (res.x - q_init) / (n_seq - 1)
            q_seq = [q_init + dq * i for i in range(n_seq)]
            vis.visualize_sequence(q_seq)
        """
        return self._create_solver_result_from_scipy_sol(sol, np_polygon, d_hover)

    # TODO: currently, movable_polygon must be len==1
    # To use multiple movable_polygon, we need to loop solve for each movable_polygon
    def solve_multiple(self, q_init, np_polygons, target_obs_pos,
                       movable_polygon=None,
                       d_hover=0.0, joint_limit_margin=0.0):
        """
        np_polygons: List[np.ndarray]
        """
        min_cost = np.inf
        min_sol = None
        target_polygon = None
        for np_polygon in np_polygons:
            try:
                sol = self.solve(
                    q_init, np_polygon, target_obs_pos,
                    movable_polygon=movable_polygon,
                    d_hover=d_hover, joint_limit_margin=joint_limit_margin)
                if sol.success and sol.fun < min_cost:
                    min_cost = sol.fun
                    min_sol = sol
                    target_polygon = np_polygon
            except ConcavePolygonException:
                print("Input polygon is not convex. Skip optimization.")
            print(sol.success)
        # TODO this causes error when min_sol is none (if all solve is fail)
        result = self._create_solver_result_from_scipy_sol(min_sol, target_polygon, d_hover)
        return result

    def _create_solver_result_from_scipy_sol(self, sol_scipy, target_polygon, d_hover):
        poses, _ = self.forward_kinematics(sol_scipy.x)
        pose = poses[0]
        return SolverResult(sol_scipy.success, sol_scipy.x, sol_scipy.fun, pose, target_polygon, d_hover, sol_scipy)
