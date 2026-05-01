import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# -----------------------------
# robot parameters
# -----------------------------
rw = 0.195/2
rb = 0.381/2

kv = 1.0
kw = 3.0

# -----------------------------
# angle wrap
# -----------------------------
def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


# -----------------------------
# drive robot (v,w → wheels)
# -----------------------------
def drive(sim, rMotor, lMotor, v, w):

    wr = (v + rb*w)/rw
    wl = (v - rb*w)/rw

    sim.setJointTargetVelocity(rMotor, wr)
    sim.setJointTargetVelocity(lMotor, wl)


# -----------------------------
# go straight to target
# -----------------------------
def go_straight(sim, robot, rMotor, lMotor, target):

    pos = sim.getObjectPosition(robot, sim.handle_world)
    ori = sim.getObjectOrientation(robot, sim.handle_world)[2]

    dx = target[0] - pos[0]
    dy = target[1] - pos[1]

    dist = math.sqrt(dx**2 + dy**2)
    theta_target = math.atan2(dy, dx)

    error = wrap_angle(theta_target - ori)

    v = kv * dist
    w = kw * error

    # limit speed
    v = max(min(v,1.0), -1.0)
    w = max(min(w,3.0), -3.0)

    wr = (v + rb*w)/rw
    wl = (v - rb*w)/rw

    sim.setJointTargetVelocity(rMotor, wr)
    sim.setJointTargetVelocity(lMotor, wl)

# -----------------------------
# vertical movement robot
# -----------------------------
def move_vertical(sim, robot, rMotor, lMotor, target_y):

    pos = sim.getObjectPosition(robot, sim.handle_world)
    ori = sim.getObjectOrientation(robot, sim.handle_world)[2]

    target = [pos[0], target_y]

    dx = target[0] - pos[0]
    dy = target[1] - pos[1]

    theta_target = math.atan2(dy, dx)
    error = wrap_angle(theta_target - ori)

    dist = abs(dy)

    if abs(error) > 0.3:
        v = 0
        w = 2 * error
    else:
        v = 1.5 * dist
        w = 0.5 * error

    v = max(min(v,1.5), -1.5)
    w = max(min(w,2.0), -2.0)

    drive(sim, rMotor, lMotor, v, w)


# -----------------------------
# connect
# -----------------------------
client = RemoteAPIClient()
sim = client.require('sim')

sim.startSimulation()
print("Simulation Started")


# -----------------------------
# objects
# -----------------------------
ball = sim.getObject("/Bola_Merah")

attacker = sim.getObject("/Robot_Pemain")
defender = sim.getObject("/Robot_Lawan_01")
goalkeeper = sim.getObject("/Robot_Lawan_02")

att_R = sim.getObject("/Robot_Pemain/rightMotor")
att_L = sim.getObject("/Robot_Pemain/leftMotor")

def_R = sim.getObject("/Robot_Lawan_01/rightMotor")
def_L = sim.getObject("/Robot_Lawan_01/leftMotor")

gk_R = sim.getObject("/Robot_Lawan_02/rightMotor")
gk_L = sim.getObject("/Robot_Lawan_02/leftMotor")

goal_right = sim.getObject("/Gwang_Kuning")
goal_pos = sim.getObjectPosition(goal_right, sim.handle_world)


# -----------------------------
# main loop
# -----------------------------
try:

    while True:

        ball_pos = sim.getObjectPosition(ball, sim.handle_world)

        # -----------------------------
        # attacker → go straight to ball
        # -----------------------------
        go_straight(sim, attacker, att_R, att_L, ball_pos)

        # -----------------------------
        # defender → vertical patrol
        # -----------------------------
        target_y = max(min(ball_pos[1],1.5), -1.5)

        move_vertical(sim, defender, def_R, def_L, target_y)

        # -----------------------------
        # goalkeeper → vertical guard
        # -----------------------------
        gk_y = max(min(ball_pos[1],0.8), -0.8)

        move_vertical(sim, goalkeeper, gk_R, gk_L, gk_y)

        time.sleep(0.05)


except KeyboardInterrupt:
    print("Stopped by user")

finally:
    sim.stopSimulation()
    print("Simulation Stopped")
