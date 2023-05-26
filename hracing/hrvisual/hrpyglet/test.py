from .bare_bone_gui import App
import random
from pyglet.gl import *
from pyglet.window import mouse
from pyglet import graphics, shapes, sprite, clock
from .visual_rigidbody import VisualRectBody, VisualCircleBody, VisualArcBody, VisualPolygonBody
from ...hrsimulation.hrphysics.rigidbody import RigidBody
from ...hrsimulation.hrphysics import rigidbody
from ...hrsimulation.hrphysics.world import World
import numpy as np

TANGENTIAL_TARGET_SPEED = 13
MAX_TARGET_SPEED = 20
TARGET_TURN_RADIUS = 500

class PhysicsTestGUI(App):

    def __init__(self, width, height, *args, **kwargs):
        
        super().__init__(width, height, *args, **kwargs)
        self.child_batch = pyglet.graphics.Batch()
        clock.schedule_interval(self.update, 0.005) # update at 60Hz
        self.world = World()
        self.world.dynamic_control = True
        self.world.add_rigid_body(VisualRectBody(0, 210, 0.55, 2.4, self.child_batch, mass=500))
        # self.arc_test2 = VisualArcBody(0, 0, 200, self.child_batch, orientation_angle=0, angle_span=np.pi/2)
        # self.world.add_rigid_body(self.arc_test2)
        # # self.world.add_rigid_body(VisualCircleBody(0, 0, 200, self.child_batch))
        # self.world.rigid_bodies[-1].is_static =  True
        self.world.add_rigid_body(VisualArcBody(0, -20, 220, self.child_batch, orientation_angle=np.pi/2, angle_span=np.pi/2))
        self.world.rigid_bodies[-1].is_static =  True
        self.time = 0
        # self.world.rigid_bodies[-1].is_static = True

    def on_mouse_press(self, x, y, button, modifiers):
        if modifiers == pyglet.window.key.MOD_SHIFT:
            mouse_x = x/self.width
            mouse_y = y/self.height

            mouse_x_in_world = self.left   + mouse_x*self.zoomed_width
            mouse_y_in_world = self.bottom + mouse_y*self.zoomed_height
            self.world.add_rigid_body(VisualRectBody(mouse_x_in_world, mouse_y_in_world, 0.55, 2.4, batch=self.child_batch))
            self.world.rigid_bodies[-1].is_static = True
            # if random.randint(0, 1):
            #     self.world.rigid_bodies[-1].is_static = True
    

    def update(self, dt):
        delta_time = 0.005
        self.time += delta_time
        move_index = 0
        force_component = 15000
        delta_movement = 3
        extra_tangential_acceleration = 0 # positive is forward
        extra_normal_accleration = 0 # positive is toward turn center
        etm = 3.34 # 0.54 * mass
        enm = 3.34
        if self.keys[pyglet.window.key.UP]:
            if move_index < len(self.world.rigid_bodies):
                if self.world.dynamic_control:    
                    # self.world.rigid_bodies[move_index].apply_force([0, force_component])
                    self.world.rigid_bodies[move_index].apply_force_in_orientation([force_component, 0])
                else:
                    self.world.rigid_bodies[move_index].move(delta_movement, 0)
            extra_tangential_acceleration = etm
        if self.keys[pyglet.window.key.LEFT]:
            if move_index < len(self.world.rigid_bodies):
                if self.world.dynamic_control:
                    # self.world.rigid_bodies[move_index].apply_force([-force_component, 0])
                    self.world.rigid_bodies[move_index].apply_force_in_orientation([0, force_component])
                else:
                    # self.world.rigid_bodies[move_index].apply_force_in_orientation([-force_component, 0])
                    self.world.rigid_bodies[move_index].move(0, delta_movement)
            extra_normal_accleration = -enm
        if self.keys[pyglet.window.key.DOWN]:
            if move_index < len(self.world.rigid_bodies):
                if self.world.dynamic_control:
                    # self.world.rigid_bodies[move_index].apply_force([0, -force_component])
                    self.world.rigid_bodies[move_index].apply_force_in_orientation([-force_component, 0])
                else:
                    self.world.rigid_bodies[move_index].move(-delta_movement, 0)
            extra_tangential_acceleration = -etm
        if self.keys[pyglet.window.key.RIGHT]:
            if move_index < len(self.world.rigid_bodies):
                if self.world.dynamic_control:
                    # self.world.rigid_bodies[move_index].apply_force([force_component, 0])
                    self.world.rigid_bodies[move_index].apply_force_in_orientation([0, -force_component])
                else:
                    self.world.rigid_bodies[move_index].move(delta_movement, 0)
            extra_normal_accleration = enm
        if self.keys[pyglet.window.key.E]:
            if move_index < len(self.world.rigid_bodies):
                self.world.rigid_bodies[move_index].rotate_radians(-np.pi/16)
        elif self.keys[pyglet.window.key.Q]:
            if move_index < len(self.world.rigid_bodies):
                self.world.rigid_bodies[move_index].rotate_radians(np.pi/16)
        


        # body = self.world.rigid_bodies[0]
        # rvect_origin_body = [-body.center_x, -body.center_y]
        # turn_radius = RigidBody.get_vector_magnitude(rvect_origin_body)
        # normal_direction = RigidBody.get_unit_vector(rvect_origin_body)
        # tangential_direction = RigidBody.transform(normal_direction, np.pi / 2)
        # set_angle = np.arctan2(tangential_direction[1], tangential_direction[0])
        # body.orientation_angle = set_angle
        # normal_velocity = np.dot(normal_direction, body.linear_velocity)
        # tangential_velocity = np.dot(tangential_direction, body.linear_velocity)
        # speed_change_rate = 0
        # if extra_tangential_acceleration == 0:
        #     speed_change_rate = TANGENTIAL_TARGET_SPEED - tangential_velocity

        # normal_acceleration_magnitude = tangential_velocity ** 2 / turn_radius
        # normal_acceleration_magnitude -= normal_velocity * 0.5
        # normal_acceleration_magnitude += extra_normal_accleration
        # tangential_acceleration_magnitude = speed_change_rate + extra_tangential_acceleration
        # if tangential_velocity >= MAX_TARGET_SPEED and tangential_acceleration_magnitude > 0:
        #     # print("no more speed change")
        #     tangential_acceleration_magnitude = 0
        # normal_acceleration = np.multiply(normal_acceleration_magnitude, normal_direction)
        # tangential_acceleration = np.multiply(tangential_acceleration_magnitude, tangential_direction)

        # total_acceleration = np.add(normal_acceleration, tangential_acceleration)

        # total_force = np.multiply(body.mass, total_acceleration)

        # Print body properties
        # print("-------")
        # print("Total Body Count:", len(self.world.rigid_bodies))
        # print("Test Arc Start Point: ", self.arc_test.start_point)
        # print("Test Arc End Point: ", self.arc_test.end_point)
        # print("Test Arc angle spane: ", self.arc_test.angle_span)
        # angle =  RigidBody.angle_vectora2b([body.center_x, body.center_y], [1, 0])
        # print("Current Time:", self.time)
        # print("Current Angle:", angle)
        # print("Normal Acceleration Magnitude:", normal_acceleration_magnitude)
        # print("Normal Velocity:", normal_velocity)
        # print("Turn Radius:", turn_radius)
        # print("Speed Change:", speed_change_rate)
        # print("Total Acceleration Magnitude: ", RigidBody.get_vector_magnitude(total_acceleration))
        # print("Total Velocity Magnitude: ", RigidBody.get_vector_magnitude(body.linear_velocity))
        # print("Tangential Velocity Magnitude:", tangential_velocity)
        # print(RigidBody.get_vector_magnitude(total_acceleration))
        # print(RigidBody.get_vector_magnitude(body.linear_velocity))
        # body.force = total_force
        # body.apply_force(total_force)

        self.world.step(delta_time)


    def on_draw(self):
        # Initialize Projection matrix
        glMatrixMode( GL_PROJECTION )
        glLoadIdentity()

        # Initialize Modelview matrix
        glMatrixMode( GL_MODELVIEW )
        glLoadIdentity()
        # Save the default modelview matrix
        glPushMatrix()

        # Clear window with ClearColor
        glClear( GL_COLOR_BUFFER_BIT )

        # Set orthographic projection matrix
        glOrtho( self.left, self.right, self.bottom, self.top, 1, -1 )
        self.child_batch.draw()
        self.fps_display.draw()
        glPopMatrix()


PhysicsTestGUI(1024, 768).run()
print("test")