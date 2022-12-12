"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""
import sys
sys.path.append('..')
sys.path.append('.')
import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import Euler2Rotation


class DrawMav:
    def __init__(self, state, window):
        """
        Draw the MAV.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.north  # north position
            state.east  # east position
            state.altitude   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        # get points that define the non-rotated, non-translated mav and the mesh colors
        self.mav_points, self.mav_meshColors = self.get_points()

        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining mav
        rotated_points = self.rotate_points(self.mav_points, R)
        translated_points = self.translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        self.mav_body = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.mav_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        window.addItem(self.mav_body)  # add body to plot

    def update(self, state):
        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining mav
        rotated_points = self.rotate_points(self.mav_points, R)
        translated_points = self.translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        # draw MAV by resetting mesh using rotated and translated points
        self.mav_body.setMeshData(vertexes=mesh, vertexColors=self.mav_meshColors)

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def get_points(self):
        """"
            Defining the points that define the aircraft mesh.
        """
        # Scalars that define the dimensions of the aircraft
        unit_length = 0.25
        plane_l = 1.7*unit_length
        wing_l = unit_length
        wing_w = unit_length/1.5
        thrust_l = 2*unit_length/3
        thrust_w = unit_length/6.5
        cone = 3*thrust_w/2
        end_w = unit_length/6
        end_l = unit_length/4
        end_inset = 3*end_l/4
        rudder_h = cone
        rudder_t = thrust_w/2
        rudder_l_b = cone
        rudder_l_t = cone/2
        front_w = 2*end_w
        front_l = front_w
        cockpit_w = 3*front_w/4
        cockpit_l = cockpit_w
        cockpit_h = 0.25*front_w

        # Points are in NED Coordinates relative to the COM of aircraft
        points = np.array([[plane_l/2, 0, 0],  # point 0
                           [plane_l/2-cockpit_l, 0, -cockpit_h],  # point 1
                           [plane_l/2-front_l, front_w/2, 0],  # point 2
                           [plane_l/2-front_l, cockpit_w/2, 0],  # point 3
                           [plane_l/2-2*cockpit_l, 0, -1.5*cockpit_h],  # point 4
                           [plane_l/2-front_l, -1*cockpit_w/2, 0],  # point 5
                           [plane_l/2-front_l, -1*front_w/2, 0],  # point 6
                           [0, 0, -0.001], #point 7
                           [-1*wing_l, wing_w, 0],  # point 8
                           [-1*wing_l, end_w/2, 0],  # point 9
                           [-1*wing_l, -1*end_w/2, 0],  # point 10
                           [-1*wing_l, -1*wing_w, 0],  # point 11
                           [-1*wing_l+end_inset, end_w/2, 0],  # point 12
                           [-1*wing_l+end_inset, 0, -1*end_w/2], #point 13
                           [-1*wing_l+end_inset, -1*end_w/2, 0],  # point 14
                           [-1*wing_l+end_inset-end_l, 0, 0],  # point 15
                           [-1*wing_l+thrust_l+cone, wing_w/2, 0],  # point 16
                           [-1*wing_l+thrust_l, wing_w/2+thrust_w/2, 0],  # point 17
                           [-1*wing_l+thrust_l, wing_w/2, -1*thrust_w/2],  # point 18
                           [-1*wing_l+thrust_l, wing_w/2-thrust_w/2, 0],  # point 19
                           [-1*wing_l+thrust_l, wing_w/2, thrust_w/2],  # point 20
                           [-1*wing_l, wing_w/2+thrust_w/2, 0],  # point 21
                           [-1*wing_l, wing_w/2, -1*thrust_w/2],  # point 22
                           [-1*wing_l, wing_w/2-thrust_w/2, 0],  # point 23
                           [-1*wing_l, wing_w/2, thrust_w/2], #point 24
                           [-1*wing_l+rudder_l_b, wing_w/2, -1*thrust_w/2],  # point 25
                           [-1*wing_l+rudder_l_b-thrust_w, wing_w/2-rudder_t, -1*thrust_w/2-rudder_h],  # point 26
                           [-1*wing_l+rudder_l_b-thrust_w-rudder_l_t, wing_w/2-rudder_t, -1*thrust_w/2-rudder_h],  # point 27
                           [-1*wing_l+thrust_l+cone, -1*wing_w/2, 0],  # point 28
                           [-1*wing_l+thrust_l, -1*wing_w/2+thrust_w/2, 0],  # point 29
                           [-1*wing_l+thrust_l, -1*wing_w/2, -1*thrust_w/2], #point 30
                           [-1*wing_l+thrust_l, -1*wing_w/2-thrust_w/2, 0],  # point 31
                           [-1*wing_l+thrust_l, -1*wing_w/2, thrust_w/2],  # point 32
                           [-1*wing_l, -1*wing_w/2+thrust_w/2, 0],  # point 33
                           [-1*wing_l, -1*wing_w/2, -1*thrust_w/2],  # point 34
                           [-1*wing_l, -1*wing_w/2-thrust_w/2, 0],  # point 35
                           [-1*wing_l, -1*wing_w/2, thrust_w/2],  # point 36
                           [-1*wing_l+rudder_l_b, -1*wing_w/2, -1*thrust_w/2], #point 37
                           [-1*wing_l+rudder_l_b-thrust_w, -1*wing_w/2+rudder_t, -1*thrust_w/2-rudder_h],  # point 38
                           [-1*wing_l+rudder_l_b-thrust_w-rudder_l_t, -1*wing_w/2+rudder_t, -1*thrust_w/2-rudder_h],  # point 39
                           ]).T

        # scale points for better rendering
        scale = 50
        points = scale * points

        #Define the Colors for Each Face of the Triangular Mesh:
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((45, 3, 4), dtype=np.float32)
        for i in range(0,45):
            meshColors[i] = np.array([0.04,0.03,0.04,0.1])
        meshColors[0] = meshColors[3] = meshColors[8] = meshColors[13] = meshColors[19] = meshColors[20] = meshColors[43] = meshColors[31] = meshColors[29] = meshColors[41] = np.array([0.74,0.,0.27,1.])
        meshColors[44] = meshColors[32] = meshColors[30] = meshColors[42] = np.array([0.25,0.,0.62,1.])
        meshColors[5] = meshColors[4] = meshColors[2] = meshColors[1] = np.array([0.,0.33,0.84,1.])

        return points, meshColors

    def points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[0], points[2], points[3]], #First Eight are Cockpit of the Plane
                        [points[0], points[1], points[3]],
                        [points[0], points[1], points[5]],
                        [points[0], points[5], points[6]],
                        [points[1], points[3], points[4]],
                        [points[1], points[4], points[5]],
                        [points[3], points[4], points[5]],
                        [points[0], points[3], points[5]], #Next Eight are the Fuselage
                        [points[2], points[3], points[12]],
                        [points[3], points[4], points[12]],
                        [points[4], points[12], points[13]],
                        [points[4], points[13], points[14]],
                        [points[4], points[5], points[14]],
                        [points[5], points[6], points[14]],
                        [points[3], points[5], points[12]],
                        [points[5], points[12], points[14]], #Next Three are the Back
                        [points[12], points[13], points[15]],
                        [points[13], points[14], points[15]],
                        [points[12], points[14], points[15]], #Next Two are the Wings
                        [points[7], points[8], points[9]],
                        [points[7], points[10], points[11]], #Next Twelve are the Right Thruster
                        [points[17], points[18], points[21]],
                        [points[18], points[21], points[22]],
                        [points[18], points[19], points[22]],
                        [points[19], points[22], points[23]],
                        [points[17], points[20], points[21]],
                        [points[20], points[21], points[24]],
                        [points[20], points[19], points[24]],
                        [points[19], points[24], points[23]],
                        [points[16], points[17], points[19]],
                        [points[16], points[18], points[20]],
                        [points[25], points[26], points[22]],
                        [points[22], points[26], points[27]], # Next Twelve are the Left Thruster
                        [points[29], points[30], points[33]],
                        [points[30], points[33], points[34]],
                        [points[30], points[34], points[35]],
                        [points[30], points[31], points[35]],
                        [points[33], points[32], points[29]],
                        [points[32], points[33], points[36]],
                        [points[31], points[36], points[32]],
                        [points[31], points[36], points[35]],
                        [points[31], points[28], points[29]],
                        [points[30], points[28], points[32]],
                        [points[38], points[37], points[34]],
                        [points[34], points[38], points[39]],
                        ])

        return mesh
