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
        self.miu_k = 0.7
        self.friction = True
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
        angle = math.atan2(vector_a[0]*vector_b[1]-vector_a[1]*vector_b[0], vector_a[0]*vector_b[0]+vector_a[1]*vector_b[1])
        return angle
    
    @staticmethod
    def get_vector_magnitude(vector):
        if vector[0] == 0 and vector[1] == 0:
            return 0
        sum = 0
        for comp in vector:
            sum += comp ** 2
        return np.sqrt(sum)

    def move(self, delta_x:float, delta_y:float):
        if not self.is_static:
            self.center_x += delta_x
            self.center_y += delta_y

    def rotate_radians(self, angle):
        self.orientation_angle += angle

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

class Polygon(RigidBody):

    def __init__(self, center_x:float, center_y:float, vertices: list[list[float]], orientation_angle: float=0, mass:float=500):
        super().__init__(center_x, center_y, orientation_angle, mass)
        self.vertices = vertices
        self.update_vertices()
    
    def step(self, delta_time, dynamic_control):
        if dynamic_control and self.friction:
            if self.is_static or (self.linear_velocity[0] == 0 and self.linear_velocity[1] == 0 and RigidBody.get_vector_magnitude(np.subtract(self.force, [0, 0])) > 0 and RigidBody.get_vector_magnitude(self.force) < self.miu_s * self.mass * 9.81):
                # print("cant move due to friction")
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
        
        transformed_vertices = []
        for vertex in self.vertices:
            vertex = RigidBody.transform(vertex, self.orientation_angle)
            vertex = np.add([self.center_x, self.center_y], vertex)
            transformed_vertices.append(vertex)
        self.transformed_vertices = transformed_vertices

    def get_collision_axis(self, body_b: RigidBody):
        res = []
        self.update_vertices()
        for end_point_index in range(len(self.transformed_vertices)):
            vector = np.subtract(self.transformed_vertices[end_point_index], self.transformed_vertices[end_point_index - 1])
            res.append(RigidBody.get_unit_vector(RigidBody.transform(vector, np.pi / 2)))
        return res
    
    def get_min_max_projection(self, unit_vector):
        self.update_vertices()
        projections = []
        for vertex in self.transformed_vertices:
            projection = np.dot(vertex, unit_vector)
            projections.append(projection)
        return [min(projections), max(projections)]

    def apply_force(self, force):
        self.force = force
    
    def apply_force_in_orientation(self, force:list[float]):
        force_transformed = RigidBody.transform(force, self.orientation_angle)
        self.apply_force(force_transformed)

    def get_aabb(self):
        self.update_vertices()
        points_x = []
        points_y = []
        for vertex in self.transformed_vertices:
            points_x.append(vertex[0])
            points_y.append(vertex[1])

        return [[min(points_x), min(points_y)], [max(points_x), max(points_y)]]


class Rect(RigidBody):

    def __init__(self, center_x, center_y, width, length, orientation_angle:float=0, mass:float=500):
        super().__init__(center_x, center_y, orientation_angle, mass)
        self.width = width
        self.length = length
        self.update_vertices()
        self.vertex_list = None
        self.area = self.length * self.width
        
    def step(self, delta_time, dynamic_control):
        if dynamic_control and self.friction:
            if self.is_static or (self.linear_velocity[0] == 0 and self.linear_velocity[1] == 0 and RigidBody.get_vector_magnitude(np.subtract(self.force, [0, 0])) > 0 and RigidBody.get_vector_magnitude(self.force) < self.miu_s * self.mass * 9.81):
                # print("cant move due to friction")
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
        self.top_right = [self.length / 2, -self.width / 2]
        self.top_left = [self.length / 2, self.width / 2]
        self.bottom_right = [-self.length / 2, -self.width / 2]
        self.bottom_left = [-self.length / 2, self.width / 2]
        self.top_right = RigidBody.transform(self.top_right, self.orientation_angle)
        self.top_left = RigidBody.transform(self.top_left, self.orientation_angle)
        self.bottom_right = RigidBody.transform(self.bottom_right, self.orientation_angle)
        self.bottom_left = RigidBody.transform(self.bottom_left, self.orientation_angle)
        self.top_right = np.add([self.center_x, self.center_y], self.top_right)
        self.top_left = np.add([self.center_x, self.center_y], self.top_left)
        self.bottom_left = np.add([self.center_x, self.center_y], self.bottom_left)
        self.bottom_right = np.add([self.center_x, self.center_y], self.bottom_right)

    def get_min_max_projection(self, unit_vector):
        # self.update_vertices()
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

class Arc(RigidBody):

    def __init__(self, center_x, center_y, radius, angle_span:float, orientation_angle:float = 0, mass= 500, is_static=True):
        super().__init__(center_x, center_y, orientation_angle, mass)
        self.radius = radius
        self.orientation_angle = orientation_angle
        self.start_point = [0, 0]
        self.end_point = [0, 0]
        if angle_span < 0:
            self.orientation_angle += angle_span
            self.angle_span = -angle_span
        else:
            self.angle_span = angle_span
        self.update_vertices()

    def step(self, delta_time, dynamic_control):
        if dynamic_control:
            if self.is_static or (self.linear_velocity[0] == 0 and self.linear_velocity[1] == 0 and RigidBody.get_vector_magnitude(self.force) < self.miu_s * self.mass * 9.81):
                # print("static friction prevent moving")
                self.update_vertices()
                return
            kinetic_friction_direction = np.multiply(-1, RigidBody.get_unit_vector(self.linear_velocity))
            kinetic_friction = np.multiply(self.miu_k * self.mass * 9.81, kinetic_friction_direction)
            self.force = np.add(self.force, kinetic_friction)
        self.linear_velocity[0] += (self.force[0] * delta_time) / self.mass
        self.linear_velocity[1] += (self.force[1] * delta_time) /self.mass
        self.center_x += self.linear_velocity[0] * delta_time
        self.center_y += self.linear_velocity[1] * delta_time
        self.force = [0, 0]
        self.update_vertices()
    
    def update_vertices(self):
        base_vector = [self.radius, 0]
        base_vector = RigidBody.transform(base_vector, self.orientation_angle)
        self.start_point = np.add([self.center_x, self.center_y], base_vector)
        base_vector = RigidBody.transform(base_vector, self.angle_span)
        self.end_point = np.add([self.center_x, self.center_y], base_vector)
        # print("start point:", self.start_point)
        # print("end point:", self.end_point)


    def get_min_max_projection(self, unit_vector: list[float]):
        self.update_vertices()
        extra_point = self.start_point
        unit_vector_angle = RigidBody.angle_vectora2b(np.subtract(self.start_point, [self.center_x, self.center_y]), unit_vector)
        rev_unit_vector = np.multiply(-1, unit_vector)
        rev_unit_vector_angle = RigidBody.angle_vectora2b(np.subtract(self.start_point, [self.center_x, self.center_y]), rev_unit_vector)
        # print("unit vector angle", unit_vector_angle)
        # print("angle span:", self.angle_span)
        if unit_vector_angle >= 0 and unit_vector_angle <= self.angle_span:
            extra_point = np.add([self.center_x, self.center_y], np.multiply(unit_vector, self.radius))
        elif rev_unit_vector_angle >= 0 and rev_unit_vector_angle <= self.angle_span:
            extra_point = np.add([self.center_x, self.center_y], np.multiply(rev_unit_vector, self.radius))
        proj_start = np.dot(self.start_point, unit_vector)
        proj_end = np.dot(self.end_point, unit_vector)
        proj_extra = np.dot(extra_point, unit_vector)
        proj_center = np.dot([self.center_x, self.center_y], unit_vector)
        res = [min(proj_start, proj_end, proj_center, proj_extra), max(proj_start, proj_end, proj_center, proj_extra)]
        return res 

    def get_collision_axis(self, relative_body:RigidBody):
        self.update_vertices()
        rel_axis = RigidBody.get_unit_vector([relative_body.center_x - self.center_x, relative_body.center_y - self.center_y])
        start_vector = np.subtract(self.start_point, [self.center_x, self.center_y])
        end_vector = np.subtract(self.end_point, [self.center_x, self.center_y])
        start_vector = RigidBody.get_unit_vector(RigidBody.transform(start_vector, -np.pi / 2))
        end_vector = RigidBody.get_unit_vector(RigidBody.transform(end_vector, np.pi / 2))
        return [rel_axis, start_vector, end_vector]

    def apply_force(self, force:list[float]):
        self.force = force
        return

    def apply_force_in_orientation(self, force:list[float]):
        force_transformed = RigidBody.transform(force, self.orientation_angle)
        self.force = force_transformed
        return  

    def get_aabb(self):
        self.update_vertices()
        start_point = self.start_point
        end_point = self.end_point
        mid_angle = self.angle_span / 2
        vecC2M = RigidBody.transform(np.subtract(self.start_point, [self.center_x, self.center_y]), mid_angle)
        mid_point = np.add([self.center_x, self.center_y], vecC2M)
        x_min = min(mid_point[0], start_point[0], end_point[0], self.center_x)
        x_max = max(mid_point[0], start_point[0], end_point[0], self.center_x)
        y_min = min(mid_point[1], start_point[1], end_point[1], self.center_y)
        y_max = max(mid_point[1], start_point[1], end_point[1], self.center_y)
        res = [[x_min, y_min], [x_max, y_max]]
        return res 
        
class Circle(RigidBody):

    def __init__(self, center_x, center_y, radius, orientaion_angle:float=0, mass:float=500):
        super().__init__(center_x, center_y, orientaion_angle, mass)
        self.radius = radius
        self.area = np.pi * self.radius ** 2
    
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
        r_vec_pos_min = np.add([self.center_x, self.center_y], np.multiply(unit_vector, self.radius))
        r_vec_pos_max = np.add([self.center_x, self.center_y], np.multiply(unit_vector, -1*self.radius))
        pos_min = np.dot(r_vec_pos_min, unit_vector)
        pos_max = np.dot(r_vec_pos_max, unit_vector)
        return [min(pos_min, pos_max), max(pos_min, pos_max)]

    def get_collision_axis(self, relative_body:RigidBody):
        vector = np.subtract([self.center_x, self.center_y], [relative_body.center_x, relative_body.center_y])
        vector_mag = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
        unit_vector = np.divide(vector, vector_mag)
        return [unit_vector]
    
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
    test_obj = Polygon(0, 0, [[-1, -2], [-3, 4], [2, 5], [3, -5]])
    print(test_obj)        