from .rigidbody import RigidBody, Rect, Circle
from .collisions import Collisions 
import numpy as np
import threading
import time
import random

class World:

    def __init__(self):
        self.rigid_bodies:list[RigidBody] = []
        self.dynamic_control = False

    def add_rigid_body(self, body: RigidBody):
        self.rigid_bodies.append(body)

    def step(self, dt):
        possible_combinations = Collisions.broad_phase(self.rigid_bodies) 
        Collisions.narrow_phase(self.rigid_bodies, possible_combinations, dt, self.dynamic_control)
        for body in self.rigid_bodies:
            body.step(dt, self.dynamic_control)

class RandWithWeight:

    def __init__(self, w: list[int]):
        self.pick_array = []
        self.prefix_sum = 0
        for index, weight in enumerate(w):
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

class setInterval :
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
        if action == 0:
            print("going up")
        elif action == 1:
            print("going down")
        elif action == 2:
            print("going left")
        elif action == 3:
            print("going right")
            test_world.rigid_bodies[1].move(-3, 0)

    # test_timer = set_interval(test_func, 0.005)

    # start action every 0.6s
    inter=setInterval(0.05,test_func)

    # will stop interval in 5s
    t=threading.Timer(5,inter.cancel)
    t.start()

    
    
    

