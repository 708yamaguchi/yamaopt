import copy
import os
import numpy as np
import time
import trimesh
import skrobot
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import rpy_matrix
from skrobot.model.primitives import MeshLink, Sphere, Axis
from skrobot.planner.utils import set_robot_config
from yamaopt.polygon_constraint import polygon_to_desired_rpy
from yamaopt.polygon_constraint import ConcavePolygonException

class VisManager:
    def __init__(self, config):
        urdf_path = os.path.expanduser(config.urdf_path)
        robot = skrobot.model.RobotModel()
        robot.load_urdf_file(urdf_path)

        self.config = config
        self.robot = robot
        self.viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
        self.add_robot(robot)
        if config.use_base:
            self.add_robot(copy.deepcopy(robot))

    def _convert_polygon_to_mesh(self, np_polygon):
        vec0 = np_polygon[1] - np_polygon[0]
        vec1 = np_polygon[2] - np_polygon[1]
        tmp = np.cross(vec0, vec1)
        n_vec = tmp / np.linalg.norm(tmp)
        center = np.mean(np_polygon, axis=0)
        #center_hover = center + n_vec -0.1

        polygon_auged = np.vstack([np_polygon, np_polygon[0]])

        faces = []
        vertices = [center]
        for i in range(len(polygon_auged)-1):
            v0 = polygon_auged[i]
            v1 = polygon_auged[i+1]
            vertices.extend([v0, v1])
            faces.append([0, len(vertices)-2, len(vertices)-1])


        vertices = np.array(vertices)
        return vertices, faces

    def add_robot(self, robot):
        self.viewer.add(robot)

    def add_polygon(
            self, np_polygon,
            flip_and_append=True, rgba=[255, 0, 0, 200], show_axis=False):
        if show_axis:
            pos = np.mean(np_polygon, axis=0)
            ypr = np.flip(polygon_to_desired_rpy(np_polygon))
            coords = Coordinates(pos, ypr)
            polygon_axis = Axis(pos=coords.worldpos(), rot=coords.worldrot())
            self.viewer.add(polygon_axis)
        if flip_and_append:
            # Currently, polygons are only visible from one side
            # Flip and append the polygon so that it is visible from both sides
            np_polygon_mirror = copy.deepcopy(np_polygon)
            np_polygon_mirror = np_polygon_mirror[::-1]
            np_polygon = [np_polygon, np_polygon_mirror]
        else:
            np_polygon = [np_polygon]
        for p in np_polygon:
            V, F = self._convert_polygon_to_mesh(p)
            mesh = visual_mesh=trimesh.Trimesh(
                    vertices=V, faces=F, face_colors=rgba)
            polygon_link = MeshLink(mesh)
            self.viewer.add(polygon_link)

    def add_target(self, target_pos):
        target_sphere_link = Sphere(0.05, pos=target_pos, color=[0, 0, 255])
        self.viewer.add(target_sphere_link)

    def reflect_solver_result(
            self, solver_result, np_polygon_list, movable_polygon=None,
            show_polygon_axis=False):
        # change visual robot configuration
        self.set_angle_vector(solver_result.x)

        # add axis indicating robot end coords to place sensor
        xyzrpy = solver_result.end_coords
        pos = xyzrpy[:3]
        ypr = np.flip(xyzrpy[3:])
        coords = Coordinates(pos, rpy_matrix(*ypr))
        hover_axis = Axis(pos=coords.worldpos(), rot=coords.worldrot())
        self.viewer.add(hover_axis)

        sensor_axis = copy.deepcopy(hover_axis)
        sensor_axis.translate([solver_result._d_hover, 0.0, 0.0])
        self.viewer.add(sensor_axis)

        # add sphere indicating optimization frame (sensing target coords)
        opt_pos = solver_result.opt_frame_coords[:3]
        opt_sphere_link = Sphere(0.05, pos=opt_pos, color=[0, 255, 0])
        self.viewer.add(opt_sphere_link)

        # add polygons which sensor will NOT be placed
        for polygon in filter(
                lambda arr: not np.array_equal(arr, solver_result.target_polygon), 
                np_polygon_list):
            self.add_polygon(polygon, show_axis=show_polygon_axis)

        # add polygon which sensor will be placed
        self.add_polygon(
            solver_result.target_polygon,
            rgba=[0, 255, 0, 255], show_axis=show_polygon_axis)

        # add polygon where robot can move
        if movable_polygon is not None:
            self.add_polygon(
                movable_polygon, rgba=[0, 255, 255, 255], show_axis=False)

    def set_angle_vector(self, q):
        joints = [self.robot.__dict__[name] for name in self.config.control_joint_names]
        set_robot_config(self.robot, joints, q, with_base=self.config.use_base)

    def show_while(self):
        self.viewer.show()

        print('==> Press [q] to close window')
        while not self.viewer.has_exit:
            time.sleep(0.1)
            self.viewer.redraw()
