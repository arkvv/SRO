# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
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
# 3. Get object handles
p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")

rw = 0.195 / 2     # wheel radius
rb = 0.381 / 2     # half wheel distance (L/2)

# %%
# Data storage
time_data = []
wr_data = []
wl_data = []
vx_data = []
omega_data = []

try:
    start_time = time.time()

    while (time.time() - start_time) < 10:
        
        current_time = time.time() - start_time
        
        # Read joint velocities
        wr_vel = sim.getJointTargetVelocity(p3dx_RW)
        wl_vel = sim.getJointTargetVelocity(p3dx_LW)

        # Correct kinematics
        vx = (wr_vel + wl_vel) * rw / 2
        omega = (wr_vel - wl_vel) * rw / (2 * rb)

        sim.addLog(1, f"Vx:{vx:.1f}m/s, Wx:{omega:.1f}rad/s")

        # Store data
        time_data.append(current_time)
        wr_data.append(wr_vel)
        wl_data.append(wl_vel)
        vx_data.append(vx)
        omega_data.append(omega)

        print(f"t={current_time:.2f}s | wr={wr_vel:.2f} wl={wl_vel:.2f}", end="\r")

        time.sleep(0.05)

finally:
    sim.stopSimulation()
    print("\nSimulation Stopped")

# %%
# ===== PLOTTING =====

# 1. Joint velocity plot
plt.figure()
plt.plot(time_data, wr_data, label="Right Wheel (rad/s)")
plt.plot(time_data, wl_data, label="Left Wheel (rad/s)")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("P3DX Joint Velocity vs Time")
plt.legend()
plt.grid()

# 2. Body velocity plot
plt.figure()
plt.plot(time_data, vx_data, label="Vx (m/s)")
plt.plot(time_data, omega_data, label="Omega (rad/s)")
plt.xlabel("Time (s)")
plt.ylabel("Velocity")
plt.title("P3DX Body Velocity vs Time")
plt.legend()
plt.grid()

plt.show()