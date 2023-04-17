import numpy as np
from abc import ABC, abstractclassmethod
import math


class RigidBody(ABC):
    
    def __init__(self, center_x, center_y, orientation_angle, mass:float = 500):
        self.center_x = center_x
        self.center_y = center_y
        self.orientation_angle = orientation_angle
        self.mass = mass
        self.area = 0
        self.linear_velocity:list[float] = [0, 0]
        self.linear_acceleration:list[float] = [0, 0]
        self.angular_velocity:float = 0
        self.angular_acceleration:float = 0
        self.force:list[float] = [0, 0]
        self.is_static = False
        self.miu_s = 0.3
        self.miu_k = 0.4
        self.friction = False
        self.moving_static = False
    
    
    @staticmethod
    def transform(vector, angle):
        rotation_matrix = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
        return np.matmul(rotation_matrix, vector)
    
    @staticmethod
    def get_unit_vector(vector):
        if vector[0] == 0 and vector[1] == 0:
            return [0, 0]
        sum = 0
        for comp in vector:
            sum += comp ** 2
        return np.divide(vector, math.sqrt(sum))
    
    @staticmethod
    def angle_vectora2b(vector_a: list[float], vector_b: list[float]):
        angle = math.degrees(math.atan2(vector_a[0]*vector_b[1]-vector_a[1]*vector_b[0],vector_a[0]*vector_b[0]+vector_a[1]*vector_b[1]))
        return angle
    
    @staticmethod
    def get_vector_magnitude(vector):
        if vector[0] == 0 and vector[1] == 0:
            return 0
        sum = 0
        for comp in vector:
            sum += comp ** 2
        return np.sqrt(sum)

    @abstractclassmethod
    def move(self, delta_x:float, delta_y:float):
        pass

    @abstractclassmethod
    def rotate_radians(self, angle):
        pass

    @abstractclassmethod
    def step(self, delta_time, dynamic_control):
        pass

    @abstractclassmethod
    def get_min_max_projection(self, unit_vector: list[float]):
        pass

    @abstractclassmethod
    def get_collision_axis(self, relative_body):
        pass

    @abstractclassmethod
    def apply_force(self, force:list[float]):
        pass

    @abstractclassmethod
    def apply_force_in_orientation(self, force:list[float]):
        pass

    @abstractclassmethod
    def get_aabb(self):
        pass


class Rect(RigidBody):

    def __init__(self, center_x, center_y, width, length, orientation_angle:float=0, mass:float=500):
        super().__init__(center_x, center_y, orientation_angle, mass)
        self.width = width
        self.length = length
        self.rotation_matrix = [[np.cos(self.orientation_angle), -np.sin(self.orientation_angle)], [np.sin(self.orientation_angle), np.cos(self.orientation_angle)]]
        self.top_right = [self.width / 2, self.length / 2]
        self.top_left = [-self.width / 2, self.length / 2]
        self.bottom_right = [self.width / 2, -self.length / 2]
        self.bottom_left = [-self.width / 2, -self.length / 2]
        self.top_right = np.matmul(self.rotation_matrix, self.top_right) 
        self.top_left = np.matmul(self.rotation_matrix, self.top_left)
        self.bottom_left = np.matmul(self.rotation_matrix, self.bottom_left)
        self.bottom_right = np.matmul(self.rotation_matrix, self.bottom_right)
        self.top_right = np.add([self.center_x, self.center_y], self.top_right)
        self.top_left = np.add([self.center_x, self.center_y], self.top_left)
        self.bottom_left = np.add([self.center_x, self.center_y], self.bottom_left)
        self.bottom_right = np.add([self.center_x, self.center_y], self.bottom_right)
        self.vertex_list = None
        self.area = self.length * self.width

    def rotate_radians(self, angle):
        self.orientation_angle += angle
        
    def step(self, delta_time, dynamic_control):
        if dynamic_control and self.friction:
            if self.is_static or (self.linear_velocity[0] == 0 and self.linear_velocity[1] == 0 and RigidBody.get_vector_magnitude(np.subtract(self.force, [0, 0])) > 0 and RigidBody.get_vector_magnitude(self.force) < self.miu_s * self.mass * 9.81):
                print("cant move due to friction")
                self.update_vertices()
                return
            kinetic_friction_direction = np.multiply(-1, RigidBody.get_unit_vector(self.linear_velocity))
            kinetic_friction = np.multiply(self.miu_k * self.mass * 9.81, kinetic_friction_direction)
            if RigidBody.get_vector_magnitude(np.subtract(self.linear_velocity, [0, 0])) <= 0.005:
                self.linear_velocity = [0, 0]
            self.force = np.add(self.force, kinetic_friction)

        self.linear_velocity[0] += (self.force[0] * delta_time) / self.mass
        self.linear_velocity[1] += (self.force[1] * delta_time) / self.mass
        # print("Resulting Linear Velocity: ", self.linear_velocity)
        self.center_x += self.linear_velocity[0] * delta_time
        self.center_y += self.linear_velocity[1] * delta_time
        self.update_vertices()
        self.force = [0, 0]


    def update_vertices(self):
        self.rotation_matrix = [[np.cos(self.orientation_angle), -np.sin(self.orientation_angle)], [np.sin(self.orientation_angle), np.cos(self.orientation_angle)]]
        self.top_right = [self.length / 2, self.width / 2]
        self.top_left = [-self.length / 2, self.width / 2]
        self.bottom_right = [self.length / 2, -self.width / 2]
        self.bottom_left = [-self.length / 2, -self.width / 2]
        self.top_right = np.matmul(self.rotation_matrix, self.top_right) 
        self.top_left = np.matmul(self.rotation_matrix, self.top_left)
        self.bottom_left = np.matmul(self.rotation_matrix, self.bottom_left)
        self.bottom_right = np.matmul(self.rotation_matrix, self.bottom_right)
        self.top_right = np.add([self.center_x, self.center_y], self.top_right)
        self.top_left = np.add([self.center_x, self.center_y], self.top_left)
        self.bottom_left = np.add([self.center_x, self.center_y], self.bottom_left)
        self.bottom_right = np.add([self.center_x, self.center_y], self.bottom_right)

    def get_min_max_projection(self, unit_vector):
        top_left = np.dot(unit_vector, self.top_left)
        top_right = np.dot(unit_vector, self.top_right)
        bottom_left = np.dot(unit_vector, self.bottom_left)
        bottom_right = np.dot(unit_vector, self.bottom_right)
        return [min(top_left, top_right, bottom_left, bottom_right), max(top_left, top_right, bottom_left, bottom_right)]
        
    def get_collision_axis(self, body_b: RigidBody):
        res = []
        first_vect = RigidBody.transform(np.subtract(self.top_right, self.top_left), np.pi / 2)
        second_vect = RigidBody.transform(np.subtract(self.bottom_right, self.top_right), np.pi / 2)
        third_vect = RigidBody.transform(np.subtract(self.bottom_left, self.bottom_right), np.pi / 2) 
        fourth_vect = RigidBody.transform(np.subtract(self.top_left, self.bottom_left), np.pi / 2)
        first_vect = RigidBody.get_unit_vector(first_vect)
        second_vect = RigidBody.get_unit_vector(second_vect)
        third_vect = RigidBody.get_unit_vector(third_vect)
        fourth_vect = RigidBody.get_unit_vector(fourth_vect)
        res.append(first_vect)
        res.append(second_vect)
        res.append(third_vect)
        res.append(fourth_vect)
        return res

    def move(self, x, y):
        self.center_x += x
        self.center_y += y
    
    def apply_force(self, force):
        self.force = force
    
    def apply_force_in_orientation(self, force:list[float]):
        force_transformed = RigidBody.transform(force, self.orientation_angle)
        self.apply_force(force_transformed)

    def get_aabb(self):
        x_min = min(self.top_left[0], self.top_right[0], self.bottom_left[0], self.bottom_right[0])
        x_max = max(self.top_left[0], self.top_right[0], self.bottom_left[0], self.bottom_right[0])
        y_min = min(self.top_left[1], self.top_right[1], self.bottom_left[1], self.bottom_right[1])
        y_max = max(self.top_left[1], self.top_right[1], self.bottom_left[1], self.bottom_right[1])

        return [[x_min, y_min], [x_max, y_max]]
        
    
class Circle(RigidBody):

    def __init__(self, center_x, center_y, radius, orientaion_angle:float=0, mass:float=500):
        super().__init__(center_x, center_y, orientaion_angle, mass)
        self.radius = radius
        self.area = np.pi * self.radius ** 2

    def rotate_radians(self, angle):
        self.orientation_angle += angle
    
    def step(self, delta_time, dynamic_contrl):
        if dynamic_contrl:
            if self.is_static or (self.linear_velocity[0] == 0 and self.linear_velocity[1] == 0 and RigidBody.get_vector_magnitude(self.force) < self.miu_s * self.mass * 9.81):
                # print("static friction prevent moving")
                return
            kinetic_friction_direction = np.multiply(-1, RigidBody.get_unit_vector(self.linear_velocity))
            kinetic_friction = np.multiply(self.miu_k * self.mass * 9.81, kinetic_friction_direction)
            self.force = np.add(self.force, kinetic_friction)
        self.linear_velocity[0] += (self.force[0] * delta_time) / self.mass
        self.linear_velocity[1] += (self.force[1] * delta_time) /self.mass
        self.center_x += self.linear_velocity[0] * delta_time
        self.center_y += self.linear_velocity[1] * delta_time
        self.force = [0, 0]

    def get_min_max_projection(self, unit_vector:list[float]):
        r_vec_pos_min = np.subtract([self.center_x, self.center_y], np.multiply(unit_vector, self.radius))
        r_vec_pos_max = np.subtract([self.center_x, self.center_y], np.multiply(unit_vector, -1*self.radius))
        pos_min = np.dot(r_vec_pos_min, unit_vector)
        pos_max = np.dot(r_vec_pos_max, unit_vector)
        return [min(pos_min, pos_max), max(pos_min, pos_max)]

    def get_collision_axis(self, relative_body:RigidBody):
        vector = np.subtract([self.center_x, self.center_y], [relative_body.center_x, relative_body.center_y])
        vector_mag = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
        unit_vector = np.divide(vector, vector_mag)
        return [unit_vector]

    def move(self, x, y):
        self.center_x += x
        self.center_y += y
    
    def apply_force(self, force):
        self.force = force

    def get_aabb(self):
        x_min = self.center_x - self.radius
        y_min = self.center_y - self.radius
        x_max = self.center_x + self.radius
        y_max = self.center_y + self.radius

        return [[x_min, y_min], [x_max, y_max]]
       
    def apply_force_in_orientation(self, force:list[float]):
        force_transformed = RigidBody.transform(force, self.orientation_angle)
        self.force = force_transformed

if __name__ == "__main__":
    print("test")
    test = Rect(0, 0, 2, 2)