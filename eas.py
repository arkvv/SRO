import time
import math
from openai import OpenAI
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

llm = OpenAI(
    base_url="http://127.0.0.1:1234/v1",
    api_key="lm-studio"
)

def choose_goal(mission_text):

    prompt = f"""
You are a robot navigation assistant.

Waypoints:

P1 = bottom left
P2 = top left
P3 = top right
P4 = bottom right

Mission:
{mission_text}

Return ONLY waypoint names separated by commas.

Examples:

P1
P2,P4
P1,P3,P2,P4

Answer:
"""

    response = llm.chat.completions.create(
        model="qwen2.5-3b-instruct",
        messages=[
            {
                "role": "user",
                "content": prompt
            }
        ],
        temperature=0
    )

    answer = response.choices[0].message.content.strip()

    print(f"\nLLM Selected: {answer}")

    return answer

rw = 0.195 / 2
rb = 0.381 / 2

kv = 0.4
kw = 2.0

goal_tolerance = 0.15
safe_distance = 0.5

def wrap_angle(a):
    return math.atan2(
        math.sin(a),
        math.cos(a)
    )


def drive(sim, rightMotor, leftMotor, v, w):

    wr = (v + rb*w)/rw
    wl = (v - rb*w)/rw

    sim.setJointTargetVelocity(
        rightMotor,
        wr
    )

    sim.setJointTargetVelocity(
        leftMotor,
        wl
    )


def go_to_point(
    sim,
    robot,
    rightMotor,
    leftMotor,
    target_pos
):

    robot_pos = sim.getObjectPosition(
        robot,
        sim.handle_world
    )

    robot_theta = sim.getObjectOrientation(
        robot,
        sim.handle_world
    )[2]

    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]

    distance = math.sqrt(
        dx**2 + dy**2
    )

    theta_target = math.atan2(
        dy,
        dx
    )

    heading_error = wrap_angle(
        theta_target - robot_theta
    )

    v = kv * distance
    w = kw * heading_error

    v = max(min(v, 0.4), -0.4)
    w = max(min(w, 1.5), -1.5)

    drive(
        sim,
        rightMotor,
        leftMotor,
        v,
        w
    )

    return distance


def min_distance(sim, sensors):

    d = 999.0

    for sensor in sensors:

        detected, dist, *_ = \
            sim.readProximitySensor(sensor)

        if detected:
            d = min(d, dist)

    return d

client = RemoteAPIClient()
sim = client.require('sim')

sim.startSimulation()

print("Simulation Started")


p3dx = sim.getObject(
    "/PioneerP3DX"
)

p3dx_rw = sim.getObject(
    "/PioneerP3DX/rightMotor"
)

p3dx_lw = sim.getObject(
    "/PioneerP3DX/leftMotor"
)


front_handles = [
    sim.getObject("/PioneerP3DX/ultrasonicSensor[3]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[4]")
]

left_handles = [
    sim.getObject("/PioneerP3DX/ultrasonicSensor[0]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[1]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[2]")
]

right_handles = [
    sim.getObject("/PioneerP3DX/ultrasonicSensor[5]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[6]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[7]")
]


waypoints = {
    "P1": sim.getObject("/P1"),
    "P2": sim.getObject("/P2"),
    "P3": sim.getObject("/P3"),
    "P4": sim.getObject("/P4")
}


user_mission = input(
    "\nEnter Mission: "
)

mission_string = choose_goal(
    user_mission
)

print(
    f"LLM Mission = {mission_string}"
)

mission = [
    wp.strip()
    for wp in mission_string.split(",")
]

current_goal = 0

print("OPEN LOOP: Move Forward")

drive(
    sim,
    p3dx_rw,
    p3dx_lw,
    0.3,
    0
)

time.sleep(0.5)

drive(
    sim,
    p3dx_rw,
    p3dx_lw,
    0,
    0
)

print("Switching to Closed Loop")

try:

    while True:

        if current_goal >= len(mission):

            print(
                "\nMission Complete!"
            )

            drive(
                sim,
                p3dx_rw,
                p3dx_lw,
                0,
                0
            )

            break

        target_name = mission[current_goal]

        target_pos = sim.getObjectPosition(
            waypoints[target_name],
            sim.handle_world
        )


        front_dist = min_distance(
            sim,
            front_handles
        )

        left_dist = min_distance(
            sim,
            left_handles
        )

        right_dist = min_distance(
            sim,
            right_handles
        )


        if front_dist < safe_distance:

            print(
                f"Avoiding Obstacle | "
                f"Front={front_dist:.2f}",
                end="\r"
            )

            if left_dist > right_dist:

                drive(
                    sim,
                    p3dx_rw,
                    p3dx_lw,
                    0.10,
                    1.2
                )

            else:

                drive(
                    sim,
                    p3dx_rw,
                    p3dx_lw,
                    0.10,
                    -1.2
                )

        else:

            distance = go_to_point(
                sim,
                p3dx,
                p3dx_rw,
                p3dx_lw,
                target_pos
            )

            print(
                f"Target={target_name} | "
                f"Distance={distance:.2f}",
                end="\r"
            )

            if distance < goal_tolerance:

                print(
                    f"\nReached {target_name}"
                )

                current_goal += 1

                time.sleep(1)

        time.sleep(0.05)


except KeyboardInterrupt:

    print(
        "\nStopped by User"
    )

finally:

    drive(
        sim,
        p3dx_rw,
        p3dx_lw,
        0,
        0
    )

    sim.stopSimulation()

    print(
        "Simulation Stopped"
    )