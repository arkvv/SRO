# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')


# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# helper function for tranformation matrix
def transformMat(alpha, beta, gamma, tx, ty, tz):
    # 1. Individual Rotation Matrices (3x3)
    rotx = np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha),  math.cos(alpha)]
        ])
    roty = np.array([
        [ math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
        ])
    rotz = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0,0,1]
        ])
    # 2. Total Rotation Matrix (R_total, 3x3)
    rot_total = np.matmul(rotx, roty)
    rot_total = np.matmul(rot_total, rotz)
    # 3. Translation Vector (t, 3x1)
    trans_vector = np.array([
                    [tx],
                    [ty],
                    [tz]
                    ])
    # 4. Create the 3x4 Transformation Matrix
    R_t_3x4  = np.hstack((rot_total, trans_vector))
    # 5. Create the Homogeneous Row [0 0 0 1] (1x4)
    homogeneous_row = np.array([[0, 0, 0, 1]])
    # 6. Vertically stack to create the 4x4 Homogeneous Transformation Matrix
    transform_matrix_4x4 = np.vstack((R_t_3x4, homogeneous_row))
    return transform_matrix_4x4


# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")
p3dx = sim.getObject("/PioneerP3DX")
p3dx_rw = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_lw = sim.getObject("/PioneerP3DX/leftMotor")
LH_Handle = sim.getObject("/LH")
perp_Handle = sim.getObject("/Perp")
path_Handle = []
for i in range(0, 40):
    path_Handle.append(sim.getObject(f"/p[{i}]"))

sensor_handles = []

for i in range(4):
    sensor_handles.append(
        sim.getObject(
            f"/PioneerP3DX/ultrasonicSensor[{i}]"
        )
    )

grid_size = 300
resolution = 0.05

occupancy_map = np.zeros((grid_size, grid_size))

rw = 0.195/2
rb = 0.318/2
d = 0.05

dt = 0.01
x_dot_int = 0.0
y_dot_int = 0.0
gamma_int = 0.0

LH_distance = 0.8

x_odom = []
y_odom = []

x_trajectory = []
y_trajectory = []

# %%
try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0.0
    while (time.time() - start_time) < 90:
        
        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
        # print(f"Running... {elapsed:.1f}s", end="\r")

        # time difference
        dt = elapsed - elapsed_prev
        elapsed_prev = elapsed

        # Get Pose of p3dx
        p3dx_position = sim.getObjectPosition(p3dx, sim.handle_world)
        p3dx_orientation = sim.getObjectOrientation(p3dx, sim.handle_world)

        # store robot trajectory
        x_trajectory.append(p3dx_position[0])
        y_trajectory.append(p3dx_position[1])

        #  Calculate LH position wrt the world
        LH_position_to_world = transformMat(0, 0, p3dx_orientation[2], p3dx_position[0], p3dx_position[1], p3dx_position[2]) @ np.array([[LH_distance], [0], [0], [1]])
        # delete 4th component
        LH_position_to_world = LH_position_to_world[:3, :]

        # Get path points positions
        path_points = []
        for i in range(len(path_Handle)):
            path_point_position = sim.getObjectPosition(path_Handle[i], sim.handle_world)
            path_points.append(path_point_position)
        
        # Create list of A->B vectors
        vec_AB = []
        for i in range(len(path_points)-1):
            A = np.array(path_points[i]).reshape((3,1))
            B = np.array(path_points[i+1]).reshape((3,1))
            vec_AB.append(B - A)
        A = np.array(path_points[-1]).reshape((3,1))
        B = np.array(path_points[0]).reshape((3,1))
        vec_AB.append(B - A)  # Close the loop
        
        # Create list of A->LH vectors
        vec_ALH = []
        for i in range(len(path_points)):
            A = np.array(path_points[i]).reshape((3,1))
            vec_ALH.append(LH_position_to_world - A)
            
        # Project ALH on AB to find scalar projection point
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
            if scalar_proj < 0:
                scalar_proj = 0
            elif scalar_proj > 1:
                scalar_proj = 1
            A = np.array(path_points[i]).reshape((3,1))
            scalar_proj_point = A + scalar_proj * vec_AB[i]
            scalar_proj_points.append(scalar_proj_point)
                    
        # Find closest scalar projection point to LH
        closest_index = 0
        min_distance = np.linalg.norm(scalar_proj_points[0] - LH_position_to_world)
        for i in range(1, len(scalar_proj_points)):
            distance = np.linalg.norm(scalar_proj_points[i] - LH_position_to_world)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        # Desired position
        desired_position = scalar_proj_points[closest_index]

        # Transformation matrix robot w.r.t world (Using Z angle rotation)
        T_world_robot = transformMat(0,0, p3dx_orientation[2], p3dx_position[0], p3dx_position[1], p3dx_position[2])
        # Desired position wrt robot
        desired_position_wrt_robot = np.linalg.inv(T_world_robot) @ np.append(desired_position, np.array([[1]]), axis=0)
        desired_position_wrt_robot = desired_position_wrt_robot[:3, :]

        # Error calculation
        x_err = desired_position_wrt_robot[0,0]
        y_err = desired_position_wrt_robot[1,0]

        ed = math.sqrt(x_err**2 + y_err**2)

        eh = math.atan2(y_err, x_err)

        # calc body speed
        vx = 0.3*ed
        wx = 0.9*eh

        # calc wheel speeds
        wr_vel = (vx + (rb*wx)/2)/rw   
        wl_vel = (vx - (rb*wx)/2)/rw

        # Actuate wheel speeds
        sim.setJointTargetVelocity(p3dx_rw, wr_vel)
        sim.setJointTargetVelocity(p3dx_lw, wl_vel)

        # Set position of LH point
        # sim.setObjectPosition(perp_Handle, sim.handle_world, scalar_proj_point.flatten().tolist())
        sim.setObjectPosition(LH_Handle, sim.handle_world, LH_position_to_world.flatten().tolist())
        sim.setObjectPosition(perp_Handle, sim.handle_world, desired_position.flatten().tolist())
        
        for sensor in sensor_handles:

            result, distance, detectedPoint, detectedObj, detectedSurf = sim.readProximitySensor(sensor)

            if result == 1:

                # sensor pose
                sensor_position = sim.getObjectPosition(
                    sensor,
                    sim.handle_world
                )

                sensor_orientation = sim.getObjectOrientation(
                    sensor,
                    sim.handle_world
                )

                # detected point in sensor frame
                point_sensor = np.array([
                    [detectedPoint[0]],
                    [detectedPoint[1]],
                    [detectedPoint[2]],
                    [1]
                ])

                # sensor -> world transformation
                T_world_sensor = transformMat(
                    sensor_orientation[0],
                    sensor_orientation[1],
                    sensor_orientation[2],
                    sensor_position[0],
                    sensor_position[1],
                    sensor_position[2]
                )

                # point in world frame
                point_world = T_world_sensor @ point_sensor

                x_world = point_world[0,0]
                y_world = point_world[1,0]

                # world -> grid index
                gx = int(x_world/resolution + grid_size//2)
                gy = int(y_world/resolution + grid_size//2)

                # bounds check
                if 0 <= gx < grid_size and 0 <= gy < grid_size:

                    occupancy_map[gy, gx] = 1

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")

plt.figure(figsize=(8,8))

# robot trajectory
plt.plot(
    x_trajectory,
    y_trajectory,
    'b-',
    linewidth=2,
    label='Robot Trajectory'
)

# exploration path
px = [p[0] for p in path_points]
py = [p[1] for p in path_points]

plt.plot(
    px,
    py,
    'ro--',
    linewidth=2,
    label='Exploration Path'
)

plt.title("Exploration Path Tracking")

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")

plt.axis('equal')
plt.grid(True)
plt.legend()

plt.figure(figsize=(8,8))

plt.imshow(
    occupancy_map,
    cmap='inferno',
    origin='lower'
)

plt.title("Occupancy Grid Mapping")

plt.xlabel("Grid X")
plt.ylabel("Grid Y")

plt.colorbar(label='Occupancy')

plt.tight_layout()

plt.show()

# %%
