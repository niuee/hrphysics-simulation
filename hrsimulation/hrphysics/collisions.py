import numpy as np
from .rigidbody import RigidBody, Circle, Rect

class Collisions:

    def __init__(self):
        pass

    @staticmethod
    def intersects(body_a:RigidBody, body_b: RigidBody):
        axis = []
        axis.extend(body_a.get_collision_axis(body_b))
        axis.extend(body_b.get_collision_axis(body_a))
        collision = True
        min_depth = float("inf")
        min_axis = axis[0]
        for proj_axis in axis:
            body_a_interval = body_a.get_min_max_projection(proj_axis)
            body_b_interval = body_b.get_min_max_projection(proj_axis)
            if body_a_interval[0] >= body_b_interval[1] or body_b_interval[0] >= body_a_interval[1]:
                collision = False
                break
            else:
                depth = abs(min(body_a_interval[1], body_b_interval[1]) - max(body_b_interval[0], body_a_interval[0]))
                if depth < min_depth:
                    min_depth = depth
                    min_axis = proj_axis
                    if body_a_interval[1] < body_b_interval[1]:
                        min_axis = np.multiply(-1, min_axis)
        if collision:
            # print("Two Objects Colliding:", collision)
            # print("Should move body A {}m in the [{}, {}] direction and body B in the [{}, {}]".format(min_depth / 2, min_axis[0], min_axis[1], -min_axis[0], -min_axis[1]))
            return [collision, min_depth, min_axis]
        else:
            return [False, None, None]

    @staticmethod
    def intersects_aabb(aabb_a:list[list[float]], aabb_b:list[list[float]]):
        # compare x axis
        if (aabb_a[0][0] <= aabb_b[1][0] and aabb_b[0][0] <= aabb_a[1][0]) or (aabb_a[0][1] <= aabb_b[1][1] and aabb_b[0][1] <= aabb_a[1][1]):
            return True
        return False
    
    @staticmethod
    def get_contact_points(body_a: RigidBody, body_b: RigidBody):
        print("placeholder")

    @staticmethod
    def broad_phase(bodies: list[RigidBody]):
        possible_combi = []
        # total_count = 0
        # skipped_count = 0
        for index in range(len(bodies)):
            for jindex in range(index + 1, len(bodies)):
                # total_count += 1
                body_a = bodies[index]
                body_b = bodies[jindex]
                body_a_aabb = body_a.get_aabb()
                body_b_aabb = body_b.get_aabb()
                if not Collisions.intersects_aabb(body_a_aabb, body_b_aabb):
                    # skipped_count += 1
                    continue
                possible_combi.append([index, jindex])
        # print("Total Broad Phase Collision Checked:", total_count)
        # print("Filtered Collision:", skipped_count)
        return possible_combi

    @staticmethod
    def narrow_phase(bodies: list[RigidBody], combinations: list[list[int]], delta_time, dynamic_control):
        for body_a_index, body_b_index in combinations:
            body_a = bodies[body_a_index]
            body_b = bodies[body_b_index]
            collide, depth, normal = Collisions.intersects(body_a, body_b)
            if collide:
                moveDisplacement = np.multiply(depth / 2, normal)
                revMoveDisplacement = np.multiply(-depth / 2, normal)
                if not body_a.is_static:
                    body_a.move(moveDisplacement[0], moveDisplacement[1])
                if not body_b.is_static:
                    body_b.move(revMoveDisplacement[0], revMoveDisplacement[1])
                if body_a.is_static:
                    body_a.move(revMoveDisplacement[0], revMoveDisplacement[1])
                    body_b.move(revMoveDisplacement[0], revMoveDisplacement[1])
                if body_b.is_static:
                    body_a.move(moveDisplacement[0], moveDisplacement[1])
                    body_b.move(moveDisplacement[0], moveDisplacement[1])

                if dynamic_control:
                    Collisions.resolve_collision(body_a, body_b, normal, depth, delta_time)

    @staticmethod
    def resolve_collision(body_a: RigidBody, body_b: RigidBody, normal:list[float], depth:float, delta_time: float):
        if body_a.is_static and body_b.is_static:
            return
        restitution = 0.4
        inverse_mass_a = 1 / body_a.mass if not body_a.is_static and not body_a.moving_static else 0
        inverse_mass_b = 1 / body_b.mass if not body_b.is_static and not body_b.moving_static else  0

        relative_velocity = np.subtract(body_a.linear_velocity, body_b.linear_velocity)
        J = -(1 + restitution) * np.dot(relative_velocity, normal)
        J /= (inverse_mass_a) + (inverse_mass_b)

        delta_a_velocity = np.multiply(J * inverse_mass_a, normal)
        delta_b_velocity = np.multiply(J * inverse_mass_b, normal)

        # change linear velocity directly
        body_a.linear_velocity[0] += delta_a_velocity[0] 
        body_a.linear_velocity[1] += delta_a_velocity[1]
        body_b.linear_velocity[0] -= delta_b_velocity[0] 
        body_b.linear_velocity[1] -= delta_b_velocity[1] 

        # change by applying force let the simulation figures out the rest
        # body_a.apply_force(np.multiply(inverse_mass_a / delta_time, [delta_a_velocity[0], delta_a_velocity[1]]))
        # body_b.apply_force(np.multiply(inverse_mass_b / delta_time, [delta_b_velocity[0], delta_b_velocity[1]]))
        # change the force by adding on to the current force instead of reseting to the solely the resolving force of the collision
        # body_a.force = np.add(body_a.force, (np.multiply(inverse_mass_a / delta_time, [delta_a_velocity[0], delta_a_velocity[1]])))
        # body_b.force = np.add(body_b.force, (np.multiply(inverse_mass_b / delta_time, [delta_b_velocity[0], delta_b_velocity[1]])))

        # body_b_initial_normal_velocity_scalar = np.dot(body_b.linear_velocity, normal)
        # body_a_inital_normal_velocity_scalar = np.dot(body_a.linear_velocity, normal)

        # body_b_final_normal_velocity_scalar = (body_a.mass * body_a_inital_normal_velocity_scalar + body_b.mass * body_b_initial_normal_velocity_scalar - 
        #                                body_a.mass * restitution * body_b_initial_normal_velocity_scalar - body_a.mass * restitution * body_a_inital_normal_velocity_scalar) / \
        #                                 (body_a.mass + body_b.mass)
        # body_a_final_normal_velocity_scalar = body_b_final_normal_velocity_scalar + restitution * body_b_initial_normal_velocity_scalar + restitution * body_a_inital_normal_velocity_scalar

        # body_a_delta_normal_velocity_scalar = body_a_final_normal_velocity_scalar - body_a_inital_normal_velocity_scalar
        # body_b_delta_normal_velocity_scalar = body_b_final_normal_velocity_scalar - body_b_initial_normal_velocity_scalar

        # body_a_delta_normal_velocity = np.multiply(body_a_delta_normal_velocity_scalar, normal)
        # body_b_delta_normal_velocity = np.multiply(-body_b_delta_normal_velocity_scalar, normal)

        # body_a.linear_velocity = np.add(body_a.linear_velocity, body_a_delta_normal_velocity)
        # body_b.linear_velocity = np.add(body_b.linear_velocity, body_b_delta_normal_velocity)

                 
if __name__ == "__main__":
    cir1 = Circle(0, 0, 30)
    cir2 = Circle(15, 0, 10)
    rect1 = Rect(0, 0, 10, 30)
    Collisions.intersects(rect1, cir2)