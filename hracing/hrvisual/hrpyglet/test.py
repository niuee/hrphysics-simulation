from .bare_bone_gui import App
import random
from pyglet.gl import *
from pyglet.window import mouse
from pyglet import graphics, shapes, sprite, clock
from .visual_rigidbody import VisualRectBody, VisualCircleBody, VisualFanBody, VisualPolygonBody, VisualCrescentBody, VisualConcaveArcBody
from ...hrsimulation.hrphysics.rigidbody import RigidBody
from ...hrsimulation.hrphysics import rigidbody
from ...hrsimulation.hrphysics.world import World
import numpy as np
import json
import os

TANGENTIAL_TARGET_SPEED = 13
MAX_TARGET_SPEED = 20
TARGET_TURN_RADIUS = 500

class RaceHorse:

    def __init__(self, name, width, length, mass):
        self.name = name
        self.width = width
        self.length = length
        self.mass = mass


class TrackHorseController:

    def __init__(self, world: World, tracks, number_of_horses, horses: list[RaceHorse], render_batch: graphics.Batch, track_width=30, turn="left", step_resolution=0.005):
        self.name = "name"
        self.tracks = tracks
        self.number_of_horses = number_of_horses
        self.horse_cur_track_index= [0 for _ in range(number_of_horses)]
        self.horse_cur_pos = [[0, 0] for _ in range(number_of_horses)]
        self.track_width = track_width
        self.turn = turn
        self.world = world
        self.step_resolution = step_resolution
        self.horses = horses
        self.batch = render_batch
        line_up_direction = self.cal_outward_normal_direction(self.tracks[0])
        starting_point = [self.tracks[0]['startPoint']['x'], self.tracks[0]['startPoint']['y']]
        spacing = 2
        starting_direction = self.cal_track_direction(self.tracks[0])
        orientation_angle = RigidBody.angle_vectora2b([1, 0], starting_direction)
        for index, horse in enumerate(self.horses):
            pos = np.add(starting_point, np.multiply(spacing * (index + 1), line_up_direction))
            self.world.add_horse(VisualRectBody(pos[0], pos[1], horse.width, horse.length, self.batch, mass = horse.mass, orientation_angle=orientation_angle))


    def step(self, delta_time):


        self.world.step(self.step_resolution)

    def exitingTrackSegement(self, horse_idx):
        track_index = self.horse_cur_track_index[horse_idx]
        if self.tracs[track_index]['trackType'] == "STRAIGHT" and \
            not self.withinStraightTrackBound(self.horse_cur_pos[horse_idx], self.tracks[track_index]):
            return True

        elif self.trac[track_index]['trackType'] == "CURVE" and \
            not self.withinCurveTracKBound(self.horse_cur_pos[horse_idx], self.tracks[track_index]):
            return True
        return False
    
    def withinStraightTrackBound(self, horse_pos, track):
        end_point = [track['endPoint']['x'], track['endPoint']['y']]
        start_point = [track['startPoint']['x'], track['startPoint']['y']]
        track_direction = RigidBody.get_unit_vector(np.subtract(end_point, start_point))
        horse_pos_direction = RigidBody.get_unit_vector(np.subtract(horse_pos, end_point))

        if self.turn == "left":
            bound_direction = RigidBody.transform(track_direction, -np.pi / 2)
            angle = RigidBody.angle_vectora2b(bound_direction, horse_pos_direction)
            if angle > 0:
                return False
        elif self.turn == "right":
            bound_direction = RigidBody.transform(track_direction, np.pi / 2)
            angle = RigidBody.angle_vectora2b(bound_direction, horse_pos_direction)
            if angle < 0:
                return False

        return True

    def withinCurveTracKBound(self, horse_pos, track):
        center = [track['center']['x'], track['center']['y']]
        end_point = [track['endPoint']['x'], track['endPoint']['y']]
        bound_direction = RigidBody.get_unit_vector(np.subtract(end_point, center))
        horse_direction = RigidBody.get_unit_vector(np.subtract(horse_pos, center))
        angle = RigidBody.angle_vectora2b(bound_direction, horse_direction)

        if self.turn == "left":
            if angle > 0:
                return False
        elif self.turn == "right":
            if angle < 0:
                return False

        return True
    
    def cal_outward_normal_direction(self, track):
        start_point = [track['startPoint']['x'], track['startPoint']['y']]
        end_point = [track['endPoint']['x'], track['endPoint']['y']]
        starting_direction = RigidBody.get_unit_vector(np.subtract(end_point, start_point))
        if self.turn == "left":
            return RigidBody.transform(starting_direction, -np.pi / 2)
        elif self.turn == "right":
            return RigidBody.transform(starting_direction, np.pi / 2)
        
    
    def cal_track_direction(self, track):
        start_point = [track['startPoint']['x'], track['startPoint']['y']]
        end_point = [track['endPoint']['x'], track['endPoint']['y']]
        return RigidBody.get_unit_vector(np.subtract(end_point, start_point))

        

class PhysicsTestGUI(App):

    def __init__(self, width, height, *args, **kwargs):
        
        super().__init__(width, height, *args, **kwargs)
        self.child_batch = pyglet.graphics.Batch()
        clock.schedule_interval(self.update, 0.005) # update at 60Hz

        self.world = World()
        # self.world.simulation_active = False
        self.world.add_rigid_body(VisualPolygonBody(0, 0, [[1.2, 0.55/2], [1.2, -0.55/2], [-1.2, -0.55/2], [-1.2, 0.55/2]], self.child_batch, mass=500))

        f = open(os.path.dirname(__file__) + '/../tracks/exp_track_8.json')
  
        # returns JSON object as 
        # a dictionary
        tracks = json.load(f)
        
        # Iterating through the json
        # list
        for track in tracks:
            # print(track)
            if track['tracktype'] == "CURVE":
                start_point = np.subtract([track['startPoint']['x'], track['startPoint']['y']], [track['center']['x'], track['center']['y']])
                orientation_angle = RigidBody.angle_vectora2b([1, 0], start_point)
                # print(f"Orientation Angle of Curve: {orientation_angle}")
                self.world.add_rigid_body(VisualCrescentBody(track['center']['x'], track['center']['y'], track['radius'], self.child_batch, orientation_angle, track['angleSpan'], is_static=True))
        test_horses = []
        for index in range(3):
            test_horses.append(RaceHorse("test horse " + str(index), 0.55, 2.4, 500))
        controller = TrackHorseController(self.world, tracks, 3, test_horses, self.child_batch)
        # Closing file
        f.close()
        
        # self.world.add_rigid_body(VisualRectBody(0, 210, 0.55, 2.4, self.child_batch, mass=500))
        # self.world.add_rigid_body(VisualPolygonBody(10, -5, [[1.2, 0.55/2], [1.2, -0.55/2], [-1.2, -0.55/2], [-1.2, 0.55/2]], self.child_batch, mass=500, is_static=True))
        # self.world.add_rigid_body(VisualPolygonBody(9.9, -4.9, [[1.2, 0.55/2], [1.2, -0.55/2], [-1.2, -0.55/2], [-1.2, 0.55/2]], self.child_batch, mass=500, is_static=True))
        self.world.add_rigid_body(VisualCrescentBody(0, -20, 220, self.child_batch, orientation_angle=0, angle_span=-np.pi/2, is_static=True))

        # self.world.add_rigid_body(VisualConcaveArcBody(0, -20, 220, self.child_batch, orientation_angle=0, angle_span=-np.pi/2, is_static=True))

        # self.add_outer_fence(0, -20, 220, self.child_batch, -np.pi/2)
        self.time = 0
    

    def add_outer_fence(self, center_x, center_y, radius, batch: graphics.Batch, angle_span:float, orientation_angle:float = 0, step_angle_deg=5, mass= 500, is_static=True):
        step_angle = np.radians(step_angle_deg)
        if angle_span < 0:
            orientation_angle += angle_span
            angle_span = -angle_span
        base_vector = [radius, 0]
        base_vector = RigidBody.transform(base_vector, orientation_angle)
        start_point = np.add([center_x, center_y], base_vector)
        extend_length = 1
        num_steps = int(angle_span // step_angle)
        for _ in range(num_steps):
            start_point = np.add([center_x, center_y], base_vector)
            adjacent_point = np.add([center_x, center_y], RigidBody.transform(base_vector, step_angle))
            opposite_point = np.add(start_point, np.multiply(RigidBody.get_unit_vector(base_vector), extend_length))
            opposite_of_adjacent_point = np.add(adjacent_point, np.multiply(RigidBody.get_unit_vector(RigidBody.transform(base_vector, step_angle)), extend_length))
            x_centroid = sum([start_point[0], adjacent_point[0], opposite_point[0], opposite_of_adjacent_point[0]]) / 4
            y_centroid = sum([start_point[1], adjacent_point[1], opposite_point[1], opposite_of_adjacent_point[1]]) / 4
            start_point = np.subtract(start_point, [x_centroid, y_centroid])
            adjacent_point = np.subtract(adjacent_point, [x_centroid, y_centroid])
            opposite_point = np.subtract(opposite_point, [x_centroid, y_centroid])
            opposite_of_adjacent_point = np.subtract(opposite_of_adjacent_point, [x_centroid, y_centroid])
            self.world.add_rigid_body(VisualPolygonBody(x_centroid, y_centroid,[start_point, adjacent_point, opposite_of_adjacent_point, opposite_point], batch, mass=mass, is_static=is_static))
            base_vector = RigidBody.transform(base_vector, step_angle)
        
        return

    def on_mouse_press(self, x, y, button, modifiers):
        if modifiers == pyglet.window.key.MOD_SHIFT:
            print("test")
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
                if self.world.simulation_active:    
                    # self.world.rigid_bodies[move_index].apply_force([0, force_component])
                    self.world.rigid_bodies[move_index].apply_force_in_orientation([force_component, 0])
                else:
                    self.world.rigid_bodies[move_index].move(0, delta_movement)
            extra_tangential_acceleration = etm
        if self.keys[pyglet.window.key.LEFT]:
            if move_index < len(self.world.rigid_bodies):
                if self.world.simulation_active:
                    # self.world.rigid_bodies[move_index].apply_force([-force_component, 0])
                    self.world.rigid_bodies[move_index].apply_force_in_orientation([0, force_component])
                else:
                    # self.world.rigid_bodies[move_index].apply_force_in_orientation([-force_component, 0])
                    self.world.rigid_bodies[move_index].move(-delta_movement, 0)
            extra_normal_accleration = -enm
        if self.keys[pyglet.window.key.DOWN]:
            if move_index < len(self.world.rigid_bodies):
                if self.world.simulation_active:
                    # self.world.rigid_bodies[move_index].apply_force([0, -force_component])
                    self.world.rigid_bodies[move_index].apply_force_in_orientation([-force_component, 0])
                else:
                    self.world.rigid_bodies[move_index].move(0, -delta_movement)
            extra_tangential_acceleration = -etm
        if self.keys[pyglet.window.key.RIGHT]:
            if move_index < len(self.world.rigid_bodies):
                if self.world.simulation_active:
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

        self.world.step(dt)


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