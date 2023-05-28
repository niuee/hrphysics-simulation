from .rigidbody import RigidBody, Rect, Circle, Polygon
from .collisions import Collisions 
import numpy as np
import threading
import time
import random

class World:

    def __init__(self):
        self.rigid_bodies:list[RigidBody] = []
        self.simulation_active = True

    def add_rigid_body(self, body: RigidBody):
        self.rigid_bodies.append(body)

    def step(self, dt):
        # print("--------")
        # print("Body Count: ", len(self.rigid_bodies))
        possible_combinations = Collisions.broad_phase(self.rigid_bodies) 
        Collisions.narrow_phase(self.rigid_bodies, possible_combinations, dt, self.simulation_active)
        for body in self.rigid_bodies:
            body.step(dt, self.simulation_active)
    
    def add_outer_fence(self, center_x, center_y, radius, angle_span:float, orientation_angle:float = 0, step_angle_deg=5, mass= 500, is_static=True):
        step_angle = np.radians(step_angle_deg)
        if angle_span < 0:
            orientation_angle += angle_span
            angle_span = -angle_span
        else:
            angle_span = angle_span
        base_vector = [radius, 0]
        base_vector = RigidBody.transform(base_vector, orientation_angle)
        start_point = np.add([center_x, center_y], base_vector)
        extend_length = 1
        num_steps = int(angle_span // step_angle)
        for _ in range(num_steps):
            adjacent_point = np.add([center_x, center_y], RigidBody.transform(base_vector, step_angle))
            opposite_point = np.add([center_x, center_y], np.multiply(RigidBody.get_unit_vector(base_vector), extend_length))
            opposite_of_adjacent_point = np.add([center_x, center_y], np.multiply(RigidBody.get_unit_vector(RigidBody.transform(base_vector, step_angle)), extend_length))
            x_centroid = sum([start_point[0], adjacent_point[0], opposite_point[0], opposite_of_adjacent_point[0]]) / 4
            y_centroid = sum([start_point[1], adjacent_point[1], opposite_point[1], opposite_of_adjacent_point[1]]) / 4
            self.add_rigid_body(Polygon(x_centroid, y_centroid,[start_point, adjacent_point, opposite_of_adjacent_point, opposite_point], mass, is_static))
            base_vector = RigidBody.transform(base_vector, step_angle)
        return


class RandWithWeight:

    def __init__(self, w: list[int]):
        self.pick_array = []
        self.prefix_sum = 0
        for _, weight in enumerate(w):
            self.prefix_sum += weight
            self.pick_array.append(self.prefix_sum)

    def pickIndex(self) -> int:
        pick = random.randint(1, self.prefix_sum)
        left = 0
        right = len(self.pick_array) - 1
        while left <= right:
            mid = left + (right - left) // 2
            if self.pick_array[mid] > pick:
                right = mid - 1
            elif self.pick_array[mid] < pick:
                left = mid + 1
            else:
                return mid
        return left

class setInterval:

    def __init__(self,interval,action) :
        self.interval=interval
        self.action=action
        self.stopEvent=threading.Event()
        self.action_space = RandWithWeight([0, 0, 0, 1, 20])
        thread=threading.Thread(target=self.__setInterval)
        thread.start()

    def __setInterval(self) :
        nextTime=time.time()+self.interval
        prev_time = time.time()
        while not self.stopEvent.wait(nextTime-time.time()) :
            nextTime+=self.interval
            delta_time = time.time() - prev_time
            prev_time = time.time()
            self.action(delta_time, self.action_space.pickIndex())

    def cancel(self) :
        self.stopEvent.set()

if __name__ == "__main__":
    test_world = World()
    test_world.add_rigid_body(Rect(0, 0, 20, 30))
    test_world.add_rigid_body(Circle(19, 0, 10))
    action_space = RandWithWeight([0,0,0,1,100])

    print("body creation")
    for _ in range(500):
        x_pos = random.random() * 1000
        y_pos = random.random() * 1000
        orientation_angle = random.random() * np.pi * 2
        test_world.add_rigid_body(Rect(x_pos, y_pos, 0.55, 2.4, orientation_angle, 500))
        test_world.rigid_bodies[-1].is_static = True
    print("body creation completed")

    # below is another method of scheduling a routine to be run at certain time interval
    # def set_interval(func, sec):
    #     def func_wrapper(**args):
    #         set_interval(func, sec)
    #         delta_time = time.time() - args["Call Time"]
    #         func(delta_time, action_space.pickIndex())
    #     t = threading.Timer(sec, func_wrapper, kwargs={"Call Time":time.time()})
    #     t.start()
    #     return t

    def test_func(delta_time, action):
        print("Delta Time is: ", delta_time)
        test_world.step(delta_time)
        # if action == 0:
        #     print("going up")
        # elif action == 1:
        #     print("going down")
        # elif action == 2:
        #     print("going left")
        # elif action == 3:
        #     print("going right")
        #     test_world.rigid_bodies[1].move(-3, 0)

    # test_timer = set_interval(test_func, 0.005)

    # start action every 0.6s
    # inter=setInterval(3,test_func)

    # will stop interval in 5s
    # t=threading.Timer(5,inter.cancel)
    # t.start()
    prev_time = time.time()
    while True:
        start_time = time.time()
        test_world.step(0.005)
        print("Delta Time:", time.time() - start_time)
        time.sleep(2)




    
    
    

