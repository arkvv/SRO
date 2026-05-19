# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require('sim')

sim.startSimulation()
print("Simulation Started")

def transformMat(alpha, beta, gamma, tx, ty, tz):

    rotx = np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha), math.cos(alpha)]
    ])

    roty = np.array([
        [math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
    ])

    rotz = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0, 0, 1]
    ])

    rot_total = rotx @ roty @ rotz

    trans_vector = np.array([
        [tx],
        [ty],
        [tz]
    ])

    Rt = np.hstack((rot_total, trans_vector))

    bottom = np.array([[0,0,0,1]])

    T = np.vstack((Rt, bottom))

    return T

p3dx = sim.getObject("/PioneerP3DX")

p3dx_rw = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_lw = sim.getObject("/PioneerP3DX/leftMotor")

sensor_handles = []

sensor_indices = [1, 4, 9, 12]

for idx in sensor_indices:
    sensor_handles.append(
        sim.getObject(
            f"/PioneerP3DX/ultrasonicSensor[{idx}]"
        )
    )

LH_Handle = sim.createDummy(0.05)
perp_Handle = sim.createDummy(0.05)

rw = 0.195 / 2
rb = 0.318 / 2

LH_distance = 0.25

vx_gain = 0.25
wx_gain = 0.8

max_vx = 0.4
max_wx = 1.5

max_wheel_speed = 5.0

path_points = [

    # RIGHT AREA
    [4.0,  2.5, 0],
    [2.0,  2.5, 0],

    [2.0,  1.0, 0],
    [4.0,  1.0, 0],

    [4.0, -1.0, 0],
    [2.0, -1.0, 0],

    [2.0, -2.5, 0],
    [4.0, -2.5, 0],

    # CENTER
    [0.5, -2.5, 0],
    [0.5,  2.5, 0],

    # LEFT AREA
    [-2.0,  2.5, 0],
    [-4.0,  2.5, 0],

    [-4.0,  1.0, 0],
    [-2.0,  1.0, 0],

    [-2.0, -1.0, 0],
    [-4.0, -1.0, 0],

    [-4.0, -2.5, 0],
    [-2.0, -2.5, 0]
]

path_Handle = []

for i, point in enumerate(path_points):

    dummy = sim.createDummy(0.03)

    sim.setObjectPosition(
        dummy,
        sim.handle_world,
        point
    )

    path_Handle.append(dummy)

grid_size = 300
resolution = 0.05

occupancy_grid = np.zeros((grid_size, grid_size))

offset = grid_size // 2

x_trajectory = []
y_trajectory = []

try:

    start_time = time.time()

    while (time.time() - start_time) < 120:

        p3dx_position = sim.getObjectPosition(
            p3dx,
            sim.handle_world
        )

        p3dx_orientation = sim.getObjectOrientation(
            p3dx,
            sim.handle_world
        )

        robot_x = p3dx_position[0]
        robot_y = p3dx_position[1]
        robot_gamma = p3dx_orientation[2]

        x_trajectory.append(robot_x)
        y_trajectory.append(robot_y)

        LH_position_to_world = (
            transformMat(
                0,
                0,
                robot_gamma,
                robot_x,
                robot_y,
                p3dx_position[2]
            )
            @ np.array([
                [LH_distance],
                [0],
                [0],
                [1]
            ])
        )

        LH_position_to_world = LH_position_to_world[:3, :]

        path_points_world = []

        for i in range(len(path_Handle)):

            path_point_position = sim.getObjectPosition(
                path_Handle[i],
                sim.handle_world
            )

            path_points_world.append(path_point_position)

        vec_AB = []

        for i in range(len(path_points_world)-1):

            A = np.array(path_points_world[i]).reshape((3,1))
            B = np.array(path_points_world[i+1]).reshape((3,1))

            vec_AB.append(B - A)

        # close loop
        A = np.array(path_points_world[-1]).reshape((3,1))
        B = np.array(path_points_world[0]).reshape((3,1))

        vec_AB.append(B - A)

        vec_ALH = []

        for i in range(len(path_points_world)):

            A = np.array(path_points_world[i]).reshape((3,1))

            vec_ALH.append(
                LH_position_to_world - A
            )

        scalar_proj_points = []

        for i in range(len(vec_AB)):

            scalar_proj = (
                np.dot(
                    vec_ALH[i].flatten(),
                    vec_AB[i].flatten()
                )
                /
                (np.linalg.norm(vec_AB[i])**2)
            )

            # clamp
            if scalar_proj < 0:
                scalar_proj = 0

            elif scalar_proj > 1:
                scalar_proj = 1

            A = np.array(
                path_points_world[i]
            ).reshape((3,1))

            scalar_proj_point = (
                A
                +
                scalar_proj * vec_AB[i]
            )

            scalar_proj_points.append(
                scalar_proj_point
            )

        closest_index = 0

        min_distance = np.linalg.norm(
            scalar_proj_points[0]
            -
            LH_position_to_world
        )

        for i in range(1, len(scalar_proj_points)):

            distance = np.linalg.norm(
                scalar_proj_points[i]
                -
                LH_position_to_world
            )

            if distance < min_distance:

                min_distance = distance
                closest_index = i

        desired_position = scalar_proj_points[closest_index]

        T_world_robot = transformMat(
            0,
            0,
            robot_gamma,
            robot_x,
            robot_y,
            p3dx_position[2]
        )

        desired_position_wrt_robot = (
            np.linalg.inv(T_world_robot)
            @ np.append(
                desired_position,
                np.array([[1]]),
                axis=0
            )
        )

        desired_position_wrt_robot = (
            desired_position_wrt_robot[:3, :]
        )

        ed = math.sqrt(
            desired_position_wrt_robot[0,0]**2
            +
            desired_position_wrt_robot[1,0]**2
        )

        eh = math.atan2(
            desired_position_wrt_robot[1,0],
            desired_position_wrt_robot[0,0]
        )

        if abs(eh) < 0.05:
            eh = 0

        vx = vx_gain * ed
        wx = wx_gain * eh

        # slow down on turns
        vx = vx * math.exp(
            -2.0 * abs(eh)
        )

        # saturation
        vx = max(-max_vx, min(max_vx, vx))
        wx = max(-max_wx, min(max_wx, wx))

        front_detected = False
        left_detected = False
        right_detected = False

        sensor_distances = []

        for i, sensor in enumerate(sensor_handles):

            result, distance, _, _, _ = sim.readProximitySensor(sensor)

            if result == 1:
                sensor_distances.append(distance)

                if distance < 0.5:

                # front-left
                    if i == 0:
                        front_detected = True
                        left_detected = True

                    # front-right
                    elif i == 1:
                        front_detected = True
                        right_detected = True

                    # left
                    elif i == 2:
                        left_detected = True

                 # right
                    elif i == 3:
                        right_detected = True

        # smooth reactive avoidance
        if front_detected:

            vx *= 0.2

            if left_detected and not right_detected:
                wx -= 1.0

            elif right_detected and not left_detected:
                wx += 1.0

            else:
                wx += 1.0

        wr_vel = (vx + rb * wx) / rw
        wl_vel = (vx - rb * wx) / rw

        wr_vel = max(
            -max_wheel_speed,
            min(max_wheel_speed, wr_vel)
        )

        wl_vel = max(
            -max_wheel_speed,
            min(max_wheel_speed, wl_vel)
        )

        sim.setJointTargetVelocity(
            p3dx_rw,
            wr_vel
        )

        sim.setJointTargetVelocity(
            p3dx_lw,
            wl_vel
        )

        sim.setObjectPosition(
            LH_Handle,
            sim.handle_world,
            LH_position_to_world.flatten().tolist()
        )

        sim.setObjectPosition(
            perp_Handle,
            sim.handle_world,
            desired_position.flatten().tolist()
        )

        for sensor in sensor_handles:

            result, distance, detectedPoint, _, _ = sim.readProximitySensor(sensor)

            if result == 1:

                sensor_matrix = sim.getObjectMatrix(
                    sensor,
                    sim.handle_world
                )

                sensor_position = sim.getObjectPosition(
                    sensor,
                    sim.handle_world
                )

                sensor_orientation = sim.getObjectOrientation(
                    sensor,
                    sim.handle_world
                )

                detected_world = (
                    transformMat(
                        sensor_orientation[0],
                        sensor_orientation[1],
                        sensor_orientation[2],
                        sensor_position[0],
                        sensor_position[1],
                        sensor_position[2]
                    )
                    @ np.array([
                        [detectedPoint[0]],
                        [detectedPoint[1]],
                        [detectedPoint[2]],
                        [1]
                    ])
                )

                wx_map = detected_world[0,0]
                wy_map = detected_world[1,0]

                gx = int(wx_map / resolution) + offset
                gy = int(wy_map / resolution) + offset

                if (
                    0 <= gx < grid_size
                    and
                    0 <= gy < grid_size
                ):

                    occupancy_grid[gy, gx] = 1

        time.sleep(0.01)

finally:

    sim.setJointTargetVelocity(p3dx_rw, 0)
    sim.setJointTargetVelocity(p3dx_lw, 0)

    sim.stopSimulation()

    print("Simulation Stopped")

    plt.figure(figsize=(8,8))

    plt.plot(
        x_trajectory,
        y_trajectory,
        linewidth=2,
        label='Robot Trajectory'
    )

    px = [p[0] for p in path_points]
    py = [p[1] for p in path_points]

    plt.plot(
        px,
        py,
        'ro--',
        label='Exploration Path'
    )

    plt.title("Exploration Path Tracking")

    plt.xlabel("X")
    plt.ylabel("Y")

    plt.axis('equal')
    plt.grid(True)
    plt.legend()

    plt.figure(figsize=(8,8))

    plt.imshow(
        occupancy_grid,
        origin='lower'
    )

    plt.title("Occupancy Grid Mapping")

    plt.xlabel("X Grid")
    plt.ylabel("Y Grid")

    plt.show()

# %%