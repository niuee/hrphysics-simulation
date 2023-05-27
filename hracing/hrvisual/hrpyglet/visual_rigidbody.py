from ...hrsimulation.hrphysics.rigidbody import RigidBody, Rect, Circle, Crescent
from ...hrsimulation.hrphysics import rigidbody
from pyglet.graphics import Batch
from pyglet import gl, shapes
from abc import abstractclassmethod, ABC
import numpy as np

RIGID_BODY_ATTRS = set({"center_x", "center_y", "orientation_angle", "mass", "linear_velocity", "linear_acceleration", "is_static", "moving_static", "radius", "miu_k", "miu_s", "force", "start_angle", "end_angle", "angle_span","start_point", "end_point"})

class VisualRigidBody(RigidBody):

    def __init__(self, center_x, center_y, orientation_angle, mass: float = 500):
        self._rigid_body:RigidBody = None
        self._visual_component: VisualComponent = None


    def get_min_max_projection(self, unit_vector:list[float]):
        return self._rigid_body.get_min_max_projection(unit_vector)
    
    def get_collision_axis(self, relative_body: RigidBody):
        return self._rigid_body.get_collision_axis(relative_body)

    def move(self, dx, dy):
        self._rigid_body.move(dx, dy)

    def rotate_radians(self, angle):
        self._rigid_body.rotate_radians(angle)
    
    def apply_force(self, force):
        self._rigid_body.apply_force(force)
    
    def set_linear_velocity(self, linear_velocity:list[float]):
        self._rigid_body.set_linear_velocity(linear_velocity)

    def increment_linear_velocity(self, delta_velocity:list[float]):
        self._rigid_body.increment_linear_velocity(delta_velocity)

    def step(self, delta_time, dynamic_contrl):
        self._rigid_body.step(delta_time, dynamic_contrl)
        self._visual_component.update_visual()

    def apply_force_in_orientation(self, force:list[float]):
        self._rigid_body.apply_force_in_orientation(force)

    def get_aabb(self):
        return self._rigid_body.get_aabb() 

    def __setattr__(self, __name: str, __value: any) -> None:
        if __name in RIGID_BODY_ATTRS:
            self._rigid_body.__dict__[__name] =  __value
        else:
            self.__dict__[__name] = __value
    
    def __getattribute__(self, __name: str) -> any:
        if __name in RIGID_BODY_ATTRS:
            return object.__getattribute__(self._rigid_body, __name)
        else:
            return object.__getattribute__(self, __name)

class VisualComponent(ABC):

    @abstractclassmethod
    def add_to_batch(self, batch:Batch):
        pass

    @abstractclassmethod
    def remove_from_batch(self):
        pass

    @abstractclassmethod
    def update_visual(self):
        pass

class VisualConcaveArcComponent(VisualComponent):
    def __init__(self, batch: Batch, center_x: float, center_y: float, ref_rigid_body: rigidbody.ConcaveArc):
        self.batch = batch
        self.vertex_list = None
        self._ref_rigid_body = ref_rigid_body
        self.add_to_batch(self.batch)

    def update_visual(self):
        if self.vertex_list == None:
            self.add_to_batch(self.batch)
            return
        points = []
        for end_vertex_index in range(1, len(self._ref_rigid_body.points)):
            points.extend(self._ref_rigid_body.points[end_vertex_index - 1].tolist())
            points.extend(self._ref_rigid_body.points[end_vertex_index].tolist())
        self.vertex_list.vertices = points

    def add_to_batch(self, batch: Batch):
        if self.vertex_list != None:
            return
        points = []
        for end_vertex_index in range(1, len(self._ref_rigid_body.points)):
            points.extend(self._ref_rigid_body.points[end_vertex_index - 1].tolist())
            points.extend(self._ref_rigid_body.points[end_vertex_index].tolist())
        count = len(self._ref_rigid_body.points) - 1
        self.vertex_list = batch.add(count * 2, gl.GL_LINES, None, ('v2d', tuple(points)), ('c3B', (235, 64, 52) * count * 2))

    def remove_from_batch(self):
        if self.vertex_list == None:
            return
        self.vertex_list.delete()
        self.vertex_list = None

class VisualPolygonComponent(VisualComponent):
    def __init__(self, batch: Batch, center_x: float, center_y: float, vertices: list[list[float]], ref_rigid_body: rigidbody.Polygon):
        self.batch = batch
        self.vertex_list = None
        self._ref_rigid_body = ref_rigid_body
        self.add_to_batch(self.batch)

    def update_visual(self):
        if self.vertex_list == None:
            self.add_to_batch(self.batch)
            return
        points = []
        for end_vertex_index in range(len(self._ref_rigid_body.transformed_vertices)):
            points.extend(self._ref_rigid_body.transformed_vertices[end_vertex_index - 1].tolist())
            points.extend(self._ref_rigid_body.transformed_vertices[end_vertex_index].tolist())
        self.vertex_list.vertices = points

    def add_to_batch(self, batch: Batch):
        if self.vertex_list != None:
            return
        points = []
        for end_vertex_index in range(len(self._ref_rigid_body.transformed_vertices)):
            points.extend(self._ref_rigid_body.transformed_vertices[end_vertex_index - 1].tolist())
            points.extend(self._ref_rigid_body.transformed_vertices[end_vertex_index].tolist())
        count = len(self._ref_rigid_body.transformed_vertices)
        self.vertex_list = batch.add(count * 2, gl.GL_LINES, None, ('v2d', tuple(points)), ('c3B', (235, 64, 52) * count * 2))

    def remove_from_batch(self):
        if self.vertex_list == None:
            return
        self.vertex_list.delete()
        self.vertex_list = None

class VisualRectComponent(VisualComponent):

    def __init__(self, batch: Batch, center_x:float, center_y:float, width: float, length:float, orientation_angle:float, ref_rigid_body: Rect):
        self.batch = batch
        self.vertex_list = None
        self._ref_rigid_body = ref_rigid_body
        self.add_to_batch(self.batch)

    def update_visual(self):
        if self.vertex_list == None:
            self.add_to_batch(self.batch)
            return
        self.points = []
        self.points.extend(self._ref_rigid_body.top_left.tolist())
        self.points.extend(self._ref_rigid_body.top_right.tolist())
        self.points.extend(self._ref_rigid_body.top_right.tolist())
        self.points.extend(self._ref_rigid_body.bottom_right.tolist())
        self.points.extend(self._ref_rigid_body.bottom_right.tolist())
        self.points.extend(self._ref_rigid_body.bottom_left.tolist())
        self.points.extend(self._ref_rigid_body.bottom_left.tolist())
        self.points.extend(self._ref_rigid_body.top_left.tolist())
        self.vertex_list.vertices = self.points

    def add_to_batch(self, batch: Batch):
        if self.vertex_list != None:
            return
        self.points = []
        self.points.extend(self._ref_rigid_body.top_left.tolist())
        self.points.extend(self._ref_rigid_body.top_right.tolist())
        self.points.extend(self._ref_rigid_body.top_right.tolist())
        self.points.extend(self._ref_rigid_body.bottom_right.tolist())
        self.points.extend(self._ref_rigid_body.bottom_right.tolist())
        self.points.extend(self._ref_rigid_body.bottom_left.tolist())
        self.points.extend(self._ref_rigid_body.bottom_left.tolist())
        self.points.extend(self._ref_rigid_body.top_left.tolist())
        self.vertex_list = batch.add(8, gl.GL_LINES, None, ('v2d', tuple(self.points)), ('c3B', (235, 64, 52) * 8))

    def remove_from_batch(self):
        if self.vertex_list == None:
            return
        self.vertex_list.delete()
        self.vertex_list = None
    
class VisualCircleComponent(VisualComponent):

    def __init__(self, batch: Batch, center_x:float, center_y:float, radius:float, ref_rigid_body):
        self.batch = batch
        self.circle = shapes.Arc(center_x, center_y, radius=radius, batch=batch)
        self.circle.color = (235, 64, 52)
        self._ref_rigid_body = ref_rigid_body
    
    def add_to_batch(self, batch:Batch):
        self.circle = shapes.Arc(self.center_x, self.center_y, radius=self.radius, batch=batch)
        self.circle.color = (235, 64, 52)

    def remove_from_batch(self):
        self.circle = None

    def update_visual(self):
        if self.circle == None:
            self.add_to_batch(self.batch)
        self.circle.x = self._ref_rigid_body.center_x
        self.circle.y = self._ref_rigid_body.center_y

class VisualFanComponent(VisualComponent):

    def __init__(self, batch: Batch, center_x: float, center_y: float, radius: float, ref_rigid_body: rigidbody.Fan, start_angle: float, angle_span:float=2 * np.pi) -> None:
        self.batch = batch
        self._ref_rigid_body = ref_rigid_body 
        self.arc = shapes.Arc(center_x, center_y, radius=radius, batch=batch, start_angle=start_angle, angle = angle_span)
        self.arc.color = (235, 64, 52)
    
    def add_to_batch(self, batch: Batch):
        self.arc = shapes.Arc(self.center_x, self.center_y, radius=self.radius, batch=batch, start_angle=self.start_angle, angle = self.end_angle - self.start_angle)
        self.arc.color = (235, 64, 52)

    def remove_from_batch(self):
        self.arc = None
    
    def update_visual(self):
        if self.arc == None:
            self.add_to_batch(self.batch)
        self.arc.x = self._ref_rigid_body.center_x
        self.arc.y = self._ref_rigid_body.center_y

class VisualCrescentComponent(VisualComponent):

    def __init__(self, batch: Batch, center_x: float, center_y: float, radius: float, ref_rigid_body: rigidbody.Crescent, start_angle: float, angle_span:float=2 * np.pi) -> None:
        self.batch = batch
        self._ref_rigid_body = ref_rigid_body 
        self.arc = shapes.Arc(center_x, center_y, radius=radius, batch=batch, start_angle=start_angle, angle = angle_span)
        self.arc.color = (235, 64, 52)
    
    def add_to_batch(self, batch: Batch):
        self.arc = shapes.Arc(self.center_x, self.center_y, radius=self.radius, batch=batch, start_angle=self.start_angle, angle = self.end_angle - self.start_angle)
        self.arc.color = (235, 64, 52)

    def remove_from_batch(self):
        self.arc = None
    
    def update_visual(self):
        if self.arc == None:
            self.add_to_batch(self.batch)
        self.arc.x = self._ref_rigid_body.center_x
        self.arc.y = self._ref_rigid_body.center_y
    
class VisualCrescentBody(VisualRigidBody):

    def __init__(self, center_x: float, center_y: float, radius: float, batch: Batch, orientation_angle:float= 0, angle_span:float = np.pi/2, mass:float = 500, is_static:bool=False):
        self._rigid_body = rigidbody.Crescent(center_x=center_x, center_y=center_y, radius=radius, orientation_angle=orientation_angle, angle_span=angle_span, mass=mass, is_static=is_static)
        self._visual_component = VisualCrescentComponent(batch, center_x=center_x, center_y=center_y, radius=radius, ref_rigid_body=self._rigid_body, start_angle=orientation_angle, angle_span=angle_span)


class VisualFanBody(VisualRigidBody):

    def __init__(self, center_x: float, center_y: float, radius: float, batch: Batch, orientation_angle:float= 0, angle_span:float = np.pi/2, mass:float = 500, is_static:bool=False):
        self._rigid_body = rigidbody.Fan(center_x=center_x, center_y=center_y, radius=radius, orientation_angle=orientation_angle, angle_span=angle_span, mass=mass, is_static=is_static)
        self._visual_component = VisualFanComponent(batch, center_x=center_x, center_y=center_y, radius=radius, ref_rigid_body=self._rigid_body, start_angle=orientation_angle, angle_span=angle_span)

class VisualConcaveArcBody(VisualRigidBody):

    def __init__(self, center_x: float, center_y: float, radius: float, batch: Batch, orientation_angle:float= 0, angle_span:float = np.pi/2, mass:float = 500, is_static:bool=False):
        self._rigid_body = rigidbody.ConcaveArc(center_x=center_x, center_y=center_y, radius=radius, orientation_angle=orientation_angle, angle_span=angle_span, mass=mass, is_static=is_static)
        self._visual_component = VisualConcaveArcComponent(batch, center_x, center_y, self._rigid_body)

class VisualCircleBody(VisualRigidBody):
    
    def __init__(self, center_x:float, center_y:float, radius:float, batch:Batch, orientation_angle=0, mass=500, is_static=False):
        self._rigid_body = Circle(center_x, center_y, radius, orientation_angle, mass, is_static)
        self._visual_component = VisualCircleComponent(batch, center_x, center_y, radius, self._rigid_body)

class VisualRectBody(VisualRigidBody):

    def __init__(self, center_x, center_y, width, length, batch:Batch, orientation_angle=0, mass=500, is_static=False):
        self._rigid_body = Rect(center_x, center_y, width, length, orientation_angle, mass, is_static)
        self._visual_component = VisualRectComponent(batch, center_x, center_y, width, length, orientation_angle, self._rigid_body)

class VisualPolygonBody(VisualRigidBody):

    def __init__(self, center_x, center_y, vertices: list[list[float]], batch: Batch, orientation_angle:float=0, mass: float = 500, is_static=False):
        self._rigid_body = rigidbody.Polygon(center_x, center_y, vertices, orientation_angle, mass, is_static)
        self._visual_component = VisualPolygonComponent(batch, center_x, center_y, vertices, self._rigid_body)

if __name__ == "__main__":
    test_batch = Batch()
    test_obj2 = VisualCircleBody(0, 0, 20, test_batch)
    test_obj = VisualRectBody(0, 0, 20, 30, test_batch)
    print(hasattr(test_obj2, 'radius'))