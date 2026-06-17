"""
Gesture Drone Control Example for AirSim

Dependencies:
pip install mediapipe
pip install opencv-python
pip install airsim

Controls:
1 Finger Up     -> Ascend
1 Finger Down   -> Descend
2 Fingers Up    -> Forward
Closed Fist     -> Hover
3 Fingers Up    -> Land

"""

import cv2
import mediapipe as mp
import airsim
import math
import time
from collections import deque, Counter

# ==============================
# SETTINGS
# ==============================

CAMERA_INDEX = 0

MOVE_SPEED = 2.0
Z_SPEED = 1.5
COMMAND_DURATION = 0.12

HOLD_KP = 0.9
HOLD_MAX_SPEED = 2.5

FAST_LAND_SPEED = 5.0
LAND_LOCK_TIMEOUT = 5.0

GESTURE_HISTORY_LENGTH = 5

# ==============================
# MEDIAPIPE SETUP
# ==============================

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    max_num_hands=1,
    model_complexity=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

gesture_history = deque(maxlen=GESTURE_HISTORY_LENGTH)

# ==============================
# HELPER FUNCTIONS
# ==============================

def distance(p1, p2):
    return math.hypot(p1.x - p2.x, p1.y - p2.y)

def get_position(client):
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    return pos.x_val, pos.y_val, pos.z_val

def is_landed(client):
    state = client.getMultirotorState()
    return state.landed_state == airsim.LandedState.Landed

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def get_direction(tip, base):
    dx = tip.x - base.x
    dy = tip.y - base.y

    if abs(dx) > abs(dy):
        return "RIGHT" if dx > 0 else "LEFT"
    else:
        return "UP" if dy < 0 else "DOWN"

def is_extended(tip, base):
    return distance(tip, base) > 0.11

def get_stable_gesture(new_gesture):
    if new_gesture:
        gesture_history.append(new_gesture)

    if len(gesture_history) == 0:
        return ""

    return Counter(gesture_history).most_common(1)[0][0]

# ==============================
# GESTURE DETECTION
# ==============================

def detect_gesture(frame):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    detected_gesture = ""

    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            lm = hand_landmarks.landmark

            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            wrist = lm[0]

            thumb_tip = lm[4]
            thumb_ip = lm[3]

            index_tip = lm[8]
            middle_tip = lm[12]
            ring_tip = lm[16]
            pinky_tip = lm[20]

            index_base = lm[5]
            middle_base = lm[9]
            ring_base = lm[13]
            pinky_base = lm[17]

            index_extended = is_extended(index_tip, index_base)
            middle_extended = is_extended(middle_tip, middle_base)
            ring_extended = is_extended(ring_tip, ring_base)
            pinky_extended = is_extended(pinky_tip, pinky_base)

            index_dir = get_direction(index_tip, index_base)
            middle_dir = get_direction(middle_tip, middle_base)
            ring_dir = get_direction(ring_tip, ring_base)

            two_fingers_close = distance(index_tip, middle_tip) < 0.10

            thumb_dx = thumb_tip.x - wrist.x
            thumb_dy = thumb_tip.y - wrist.y

            thumb_right = thumb_dx > 0.16 and abs(thumb_dx) > abs(thumb_dy)
            thumb_down = thumb_dy > 0.16 and abs(thumb_dy) > abs(thumb_dx)

            fingers_folded = (
                not index_extended and
                not middle_extended and
                not ring_extended and
                not pinky_extended
            )

            three_fingers_up = (
                index_extended and
                middle_extended and
                ring_extended and
                not pinky_extended and
                index_dir == "UP" and
                middle_dir == "UP" and
                ring_dir == "UP"
            )

            if three_fingers_up:
                detected_gesture = "LAND"

            elif fingers_folded and thumb_right:
                detected_gesture = "RIGHT"

            elif fingers_folded and thumb_down:
                detected_gesture = "DOWN"

            elif fingers_folded:
                detected_gesture = "HOLD"

            elif index_extended and middle_extended and two_fingers_close and not ring_extended and not pinky_extended:
                if index_dir == "UP" and middle_dir == "UP":
                    detected_gesture = "FRONT"
                elif index_dir == "DOWN" and middle_dir == "DOWN":
                    detected_gesture = "BACK"

            elif index_extended and not middle_extended and not ring_extended and not pinky_extended:
                if index_dir == "UP":
                    detected_gesture = "UP"
                elif index_dir == "LEFT":
                    detected_gesture = "LEFT"

    return get_stable_gesture(detected_gesture)

# ==============================
# MOVEMENT CONTROL
# ==============================

def gesture_to_velocity(gesture):
    vx, vy, vz = 0, 0, 0

    if gesture == "UP":
        vz = -Z_SPEED

    elif gesture == "DOWN":
        vz = Z_SPEED

    elif gesture == "FRONT":
        vx = MOVE_SPEED

    elif gesture == "BACK":
        vx = -MOVE_SPEED

    elif gesture == "RIGHT":
        vy = MOVE_SPEED

    elif gesture == "LEFT":
        vy = -MOVE_SPEED

    return vx, vy, vz

def hold_position(client, hold_x, hold_y, hold_z):
    current_x, current_y, current_z = get_position(client)

    vx = clamp((hold_x - current_x) * HOLD_KP, -HOLD_MAX_SPEED, HOLD_MAX_SPEED)
    vy = clamp((hold_y - current_y) * HOLD_KP, -HOLD_MAX_SPEED, HOLD_MAX_SPEED)
    vz = clamp((hold_z - current_z) * HOLD_KP, -HOLD_MAX_SPEED, HOLD_MAX_SPEED)

    client.moveByVelocityAsync(vx, vy, vz, COMMAND_DURATION)

def fast_land(client):
    client.moveByVelocityAsync(0, 0, FAST_LAND_SPEED, 0.4).join()
    client.landAsync()

# ==============================
# AIRSIM SETUP
# ==============================

print("Connecting to AirSim...")

client = airsim.MultirotorClient()
client.confirmConnection()

client.enableApiControl(True)
client.armDisarm(True)

print("Camera started.")
print("Drone will NOT take off automatically.")
print("UP gesture = takeoff / move up.")
print("3 fingers up = fast landing.")
print("P = land and close.")

# ==============================
# CAMERA LOOP
# ==============================

cap = cv2.VideoCapture(CAMERA_INDEX)

hold_position_saved = False
hold_x, hold_y, hold_z = 0, 0, 0
last_gesture = ""

landing_started = False
landing_start_time = 0

while True:
    ret, frame = cap.read()

    if not ret:
        print("Camera not detected")
        break

    frame = cv2.flip(frame, 1)

    gesture = detect_gesture(frame)
    landed = is_landed(client)

    # ==============================
    # LANDING MODE
    # ==============================

    if landing_started:
        gesture = "LANDING..."

        elapsed_landing_time = time.time() - landing_start_time

        if landed:
            landing_started = False
            hold_position_saved = False
            gesture_history.clear()
            gesture = "LANDED - SHOW UP TO TAKEOFF"
            print("Drone landed. Commands unlocked.")

        elif elapsed_landing_time >= LAND_LOCK_TIMEOUT:
            landing_started = False
            hold_position_saved = False
            gesture_history.clear()
            gesture = "LAND TIMEOUT - COMMANDS UNLOCKED"
            print("Landing timeout reached. Commands unlocked.")

        else:
            client.moveByVelocityAsync(0, 0, FAST_LAND_SPEED, 0.15)

            cv2.putText(
                frame,
                f"LANDING... COMMANDS LOCKED {int(LAND_LOCK_TIMEOUT - elapsed_landing_time)}s",
                (30, 130),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                3
            )

    # ==============================
    # NORMAL MODE
    # ==============================

    else:
        if landed:
            hold_position_saved = False

            if gesture == "UP":
                print("UP detected. Taking off...")
                client.takeoffAsync().join()
                client.moveByVelocityAsync(0, 0, -Z_SPEED, 0.5)

            elif gesture == "LAND":
                gesture = "ALREADY LANDED"

            else:
                gesture = "LANDED - SHOW UP TO TAKEOFF"

        else:
            if gesture == "LAND":
                landing_started = True
                landing_start_time = time.time()
                hold_position_saved = False
                gesture_history.clear()
                print("Fast landing started. Commands locked.")
                fast_land(client)

            elif gesture == "HOLD":
                if last_gesture != "HOLD" or not hold_position_saved:
                    hold_x, hold_y, hold_z = get_position(client)
                    hold_position_saved = True
                    print(
                        "Hold position saved:",
                        round(hold_x, 2),
                        round(hold_y, 2),
                        round(hold_z, 2)
                    )

                hold_position(client, hold_x, hold_y, hold_z)

            else:
                hold_position_saved = False
                vx, vy, vz = gesture_to_velocity(gesture)
                client.moveByVelocityAsync(vx, vy, vz, COMMAND_DURATION)

    last_gesture = gesture

    cv2.putText(
        frame,
        f"GESTURE: {gesture}",
        (30, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.1,
        (0, 255, 0),
        3
    )

    if hold_position_saved and gesture == "HOLD":
        cv2.putText(
            frame,
            f"HOLD POS: X={hold_x:.2f} Y={hold_y:.2f} Z={hold_z:.2f}",
            (30, 125),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2
        )

    cv2.imshow("AirSim Gesture Drone Control", frame)

    if cv2.waitKey(1) & 0xFF == ord("p"):
        print("P pressed. Landing and closing...")
        if not is_landed(client):
            client.moveByVelocityAsync(0, 0, FAST_LAND_SPEED, 0.5).join()
            client.landAsync().join()
        break

# ==============================
# CLEAN EXIT
# ==============================

client.armDisarm(False)
client.enableApiControl(False)

cap.release()
cv2.destroyAllWindows()

print("Program closed.")
