from .hrphysics.rigidbody import RigidBody, Rect, Circle
from pyglet.graphics import Batch
from pyglet import gl, shapes
from abc import abstractclassmethod, ABC

RIGID_BODY_ATTRS = set({"center_x", "center_y", "orientation_angle", "mass", "linear_velocity", "linear_acceleration", "is_static", "moving_static", "radius", "miu_k", "miu_s", "force"})

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


class VisualCircleBody(VisualRigidBody):
    
    def __init__(self, center_x:float, center_y:float, radius:float, batch:Batch, orientation_angle=0, mass=500):
        self._rigid_body = Circle(center_x, center_y, radius, orientation_angle, mass)
        self._visual_component = VisualCircleComponent(batch, center_x, center_y, radius, self._rigid_body)


class VisualRectBody(VisualRigidBody):

    def __init__(self, center_x, center_y, width, length, batch:Batch, orientation_angle=0, mass=500):
        self._rigid_body = Rect(center_x, center_y, width, length, orientation_angle, mass)
        self._visual_component = VisualRectComponent(batch, center_x, center_y, width, length, orientation_angle, self._rigid_body)

    # def __setattr__(self, __name: str, __value: any) -> None:
    #     if __name in RIGID_BODY_ATTRS:
    #         self._rigid_body.__dict__[__name] =  __value
    #     else:
    #         self.__dict__[__name] = __value
    
    # def __getattribute__(self, __name: str) -> any:
    #     if __name in RIGID_BODY_ATTRS:
    #         return self._rigid_body.__dict__[__name]
    #     else:
    #         return object.__getattribute__(self, __name)

if __name__ == "__main__":
    test_batch = Batch()
    test_obj2 = VisualCircleBody(0, 0, 20, test_batch)
    test_obj = VisualRectBody(0, 0, 20, 30, test_batch)
    print(hasattr(test_obj2, 'radius'))