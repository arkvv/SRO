# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# %%
# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# %%
# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")

p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")

rw = 0.195/2
rb = 0.381/2
d = 0.05

dt = 0.05
x_dot_int = 0.0
y_dot_int = 0.0
gamma_int = 0.0

x_odom = []
y_odom = []

x_orient_int = 0.0
y_orient_int = 0.0

x_orient = []
y_orient = []

p3dx = sim.getObject("/PioneerP3DX")

# %%
try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0.0
    while (time.time() - start_time) < 30:
        
        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
        print(f"Running... {elapsed:.1f}s", end="\r")
        
        dt = elapsed - elapsed_prev
        elapsed_prev = elapsed

        wr_vel = sim.getJointTargetVelocity(p3dx_RW)
        wl_vel = sim.getJointTargetVelocity(p3dx_LW)

        vx = (wr_vel + wl_vel)*rw/2
        wx = (wr_vel - wl_vel)*rw/rb

        euler_angle = sim.getObjectOrientation(p3dx, sim.handle_world)
        
        x_dot = vx*math.cos(gamma_int)
        y_dot = vx*math.sin(gamma_int)

        x_dot_int = x_dot_int + x_dot*dt
        y_dot_int = y_dot_int + y_dot*dt
        gamma_int = gamma_int + wx*dt

        theta = euler_angle[2]

        x_dot_orient = vx * math.cos(theta)
        y_dot_orient = vx * math.sin(theta)

        x_orient_int += x_dot_orient * dt
        y_orient_int += y_dot_orient * dt

        x_orient.append(x_orient_int)
        y_orient.append(y_orient_int)   

        sim.addLog(1, f"vx:{vx:.1f}m/s, wx:{wx:.1f}rad/s, x_dot:{x_dot:.1f}m/s, y_dot:{y_dot:.1f}m/s")

        x_odom.append(x_dot_int)
        y_odom.append(y_dot_int)

        time.sleep(0.1) 

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")

plt.figure(figsize=(6,6))

plt.plot(x_orient, y_orient, 'r-', label='Orientation Data')
plt.plot(x_odom, y_odom, 'b--', label='Angular Velocity Integration')

plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Pioneer P3DX Spatial Path')
plt.legend()
plt.grid(True)
plt.axis('equal')

plt.show()

# plt.figure(figsize=(10,4))
# plt.plot(x_odom, y_odom, 'b-', label='Odometry')
# plt.xlabel('X Position (m)')
# plt.ylabel('Y Position (m)')
# plt.legend()
# plt.grid(True)
# plt.show()
