import numpy as np
from pynput import keyboard
import time

# Declare globabl flags
global t, pos_h, input_flag

# Delare state variables
x = 0; y = 0; theta = 0
# fowards, back, left, right, stop
payload = ['F', 'B', 'L', 'R',  'S']

# Initialize Homogenous Coordinates
pos_h = np.array([[0, 1, 0], [0, 0, 1], [1, 1, 1]])

# Motor actuation model paramters
left_spin_rate = 2.606      # rad/s
right_spin_rate = 2.548     # rad/s
fwd_move_rate = 121.4       # mm/s
bkwd_move_rate = 120.9      # mm/s

key_map = {
    keyboard.Key.up : 'F',
    keyboard.Key.down : 'B',
    keyboard.Key.left : 'L',
    keyboard.Key.right : 'R',
}

## Upon button press
def on_press(key):
    global t, pos_h, input_flag

    # if a valid keystroke is detected, begin timer
    if key == keyboard.Key.up or keyboard.Key.down or keyboard.Key.left or keyboard.Key.right:
        if input_flag == False:
            print_state(pos_h, get_heading(pos_h), key_map[key])
            t = time.time()
            input_flag = True

## Upon button release
def on_release(key):
    global t, pos_h, input_flag

    ti1 = time.time() - t #converting float to str, slicing the float
    input_flag = False

    # get current state
    x = pos_h[0,0]
    y = pos_h[1,0]
    theta = get_heading(pos_h)

    # in the case that the input is "forwards"
    if key == keyboard.Key.up:
        # the complete forwards transform is given by:
        # Rotation of body axis by -theta, aligning with world axis
        # Move forwards by an amount determined by the fwd_move_rate
        # Rotation of body axis by theta, to restore the heading of the body axis
        transform = np.matmul(rot_array(theta),np.matmul(trans_array(ti1 * fwd_move_rate,0),rot_array(-theta)))

        pos_h = np.matmul(transform,pos_h)
        print_state(pos_h, theta, "S")

    # in the case that the input is "backwards"
    if key == keyboard.Key.down:
        # the complete backwards transform is given by:
        # Rotation of body axis by -theta, aligning with world axis
        # Move backwards by an amount determined by the bkwd_move_rate
        # Rotation of body axis by theta, to restore the heading of the body axis
        transform = np.matmul(rot_array(theta),np.matmul(trans_array(-ti1 * bkwd_move_rate,0),rot_array(-theta)))

        pos_h = np.matmul(transform,pos_h)
        print_state(pos_h, theta, "S")

    if key == keyboard.Key.left:
        # the complete left rotation transform is given by:
        # Align origin of body axis with world axis
        # Spin left by an amount determined by the left_spin_rate
        # Restore the origin of the body axis to the robot's position
        rot = rot_array(ti1*left_spin_rate)
        transform = np.matmul(trans_array(x,y),np.matmul(rot,trans_array(-x,-y)))

        pos_h = np.matmul(transform,pos_h)
        print_state(pos_h, theta, "S")

    if key == keyboard.Key.right:
        # the complete right rotation transform is given by:
        # Align origin of body axis with world axis
        # Spin left by an amount determined by the right_spin_rate
        # Restore the origin of the body axis to the robot's position
        rot = rot_array(-ti1*right_spin_rate)
        transform = np.matmul(trans_array(x,y),np.matmul(rot,trans_array(-x,-y)))

        pos_h = np.matmul(transform,pos_h)
        print_state(pos_h, theta, "S")


    if key == keyboard.Key.esc:
        # Step simulation
        return False


## -----------------------------
## Transformation matrices

# Linear translation
def trans_array(tx,ty):
    mat = np.array([[1, 	0,		tx],
            		[0, 	1, 		ty],
            		[0,     0,      1]])
    return mat

# Rotation
def rot_array(dtheta):
    mat = np.array([[np.cos(dtheta), 	-np.sin(dtheta),    0],
                    [np.sin(dtheta), 	np.cos(dtheta), 	0],
                	[0, 				0, 					1]])
    return mat

## -----------------------------
## Helper functions
# Get current heading
def get_heading(pos_h):
    return np.arccos(pos_h[0,1] - pos_h[0,0])

# Print current state to the console
def print_state(pos_h, theta, cmd):
    print(str(cmd) + " |  x: " + str(pos_h[0,0])[0:5] + " (mm), y:" + str(pos_h[1,0])[0:5] + " (mm), heading:" + str(theta*180/np.pi)[0:5] + " deg")


## -----------------------------
## Track inputs
# Begin timer
t = time.time()
input_flag = False

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release ) as listener:
   listener.join()
