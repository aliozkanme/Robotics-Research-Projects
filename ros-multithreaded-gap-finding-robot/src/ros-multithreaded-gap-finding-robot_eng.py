#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=====================================================================================
                    TECHNICAL CONFIGURATION EXPLANATION
=====================================================================================
Line 1 (Shebang - #!/usr/bin/env python3):
   - This line indicates to the Linux/ROS environment that this file is a "Python 3"
     script and must be executed with the appropriate interpreter.
   - The '/usr/bin/env' command dynamically locates the Python 3 path in the system,
     ensuring the code runs seamlessly across different computers (portability).
   - Thanks to this line, the file can be executed directly via './filename.py' 
     or the 'rosrun' command.

Line 2 (Encoding - # -*- coding: utf-8 -*-):
   - Specifies to the Python interpreter that the file's character encoding is UTF-8.
   - This configuration ensures that special characters can be used within the code 
     and comments without raising errors or causing corruption.
=====================================================================================

=====================================================================================
"""

#####################################################################################
#####################################################################################
##                                                                                 ##
## PROJECT      : LIDAR SENSOR-BASED AUTONOMOUS OBSTACLE AVOIDANCE ROBOT DESIGN    ##
##                IN ROS AND GAZEBO SIMULATION ENVIRONMENT                         ##
## ARCHITECTURE : MULTITHREADED & REACTIVE CONTROL                                 ##
## DEVELOPER    : ALI OZKAN                                                        ##
##                                                                                 ##
#####################################################################################
#####################################################################################

#####################################################################################
## LICENSE   : CC BY-NC-SA 4.0 (Attribution-NonCommercial-ShareAlike)              ##
## COPYRIGHT : 2026, Ali Özkan                                                     ##
## NOTICE    : This code cannot be used for commercial purposes.                   ##
#####################################################################################

"""
=====================================================================================
                                    DESCRIPTION
=====================================================================================
This code includes "Gap Finding" and "Dynamic Velocity" algorithms. Additionally, 
to enhance user experience, ROS communication and Navigation logic are executed 
in separate threads.
=====================================================================================

=====================================================================================
"""

# --- IMPORTING LIBRARIES ---
import time
import threading
import math
import sys
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# --- GLOBAL VARIABLES AND STATE FLAGS ---
global scan_data        # Global variable holding Lidar data
global is_navigating    # Robot movement state (True/False)
global system_active    # Main loop status of the program
global thread_lock      # Lock to prevent data read/write race conditions

scan_data = None
is_navigating = False
system_active = True
thread_lock = threading.Lock()

# --- ROBOT PHYSICAL PARAMETERS ---
ROBOT_WIDTH = 0.50          # Robot width (Including safety margin - meters)
MAX_SPEED_LINEAR = 0.26     # Maximum linear velocity (m/s)
MAX_SPEED_ANGULAR = 1.82    # Maximum angular velocity (rad/s)
MIN_STOP_DIST = 0.25        # Emergency stop distance (m)
GAP_THRESHOLD = 0.80        # Depth required for an interval to be considered a "gap" (m)

# --- INITIALIZING ROS NODE ---
rospy.init_node('otonom_navigasyon', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
move = Twist()


# --- MATHEMATICAL FUNCTIONS (Algorithm Details) ---

def calculate_alpha(distance):
    """
    [Project Equation]: Calculates the minimum angular width (alpha) required 
    for the robot to pass through obstacles at a certain distance.
    
    Formula: alpha = 2 * arctan( Robot_Width / (2 * Distance) )
    """
    if distance <= 0: 
        return 90.0 # Return max angle if distance is 0 (Error protection)
    
    # Calculation in radians
    alpha_rad = 2 * math.atan(ROBOT_WIDTH / (2 * distance))
    
    # Convert to degrees
    alpha_deg = math.degrees(alpha_rad)
    
    # Adding +1 degree as safety margin (Thesis recommendation)
    return alpha_deg + 1.0


def calculate_dynamic_velocity(front_distance):
    """
    [Project Equation]: Reduces speed exponentially rather than linearly as the 
    robot approaches an obstacle. This ensures the robot stops smoothly.
    
    Formula: V = V_max * (1 - exp( -k * (d - d_min) ))
    """
    if front_distance < MIN_STOP_DIST:
        return 0.0
    
    # Velocity profile calculation
    # Coefficient (1.0) determines braking stiffness
    velocity = MAX_SPEED_LINEAR * (1 - math.exp(-(front_distance - MIN_STOP_DIST)))
    
    # Saturation
    if velocity > MAX_SPEED_LINEAR:
        velocity = MAX_SPEED_LINEAR
    elif velocity < 0.0:
        velocity = 0.0
        
    return velocity


def get_clean_scan_data(raw_msg):
    """
    Cleans raw data coming from the Lidar sensor.
    1. Cleans infinite (inf) values.
    2. Cleans erroneous (NaN) values.
    3. Filters very close noise values (under 0.15m).
    """
    clean_ranges = []
    for r in raw_msg.ranges:
        if math.isinf(r) or math.isnan(r):
            clean_ranges.append(3.5) # Assign max range
        elif r < 0.15:
            # Lidar sometimes detects the robot's own cables; to prevent this,
            # we consider very close values as "far" (Filtering).
            clean_ranges.append(3.5) 
        else:
            clean_ranges.append(r)
    return clean_ranges


def find_best_gap(ranges):
    """
    [Gap Finding Algorithm]: Searches for the widest and deepest gap around the robot.
    It does not just look left or right; it scans the entire panorama to find the safest route.
    
    Return: (Target Angle [Radians], Target Distance [Meters])
    """
    # Merging right and left sectors for processing convenience.
    # Turtlebot3 Lidar Structure: [0-90] Left, [270-360] Right.
    # Combined Panorama: [Right Side (90 deg)] + [Left Side (90 deg)]
    panorama = ranges[270:360] + ranges[0:90]
    
    max_score = -1          # Max gap score
    best_angle_index = -1   # Midpoint of the best gap
    current_gap_start = -1  # Gap start index
    
    # Scan panorama from start to finish
    for i in range(len(panorama)):
        dist = panorama[i]
        
        # If distance is greater than threshold, this is a "gap".
        if dist > GAP_THRESHOLD:
            if current_gap_start == -1:
                current_gap_start = i # A new gap started
        else:
            # Gap ended or obstacle detected
            if current_gap_start != -1:
                # Analyze and score the gap
                gap_width = i - current_gap_start
                
                # Heuristic Scoring: Width * Average Depth
                # (Wide and deep gaps get higher scores)
                avg_depth = sum(panorama[current_gap_start:i]) / gap_width
                score = gap_width * avg_depth
                
                if score > max_score:
                    max_score = score
                    # Select the exact center of the gap as target
                    best_angle_index = current_gap_start + (gap_width // 2)
                
                current_gap_start = -1 # Reset counter

    # If no suitable gap is found (e.g., Dead End)
    if best_angle_index == -1:
        return None, 0.0
    
    # Convert found index to angle relative to robot
    # Panorama indexed 0..180. Index 90 is directly in front (0 degrees).
    # Index 0 = -90 degrees (Right), Index 180 = +90 degrees (Left).
    target_angle_deg = best_angle_index - 90
    target_dist = panorama[best_angle_index]
    
    return math.radians(target_angle_deg), target_dist


# --- THREADS ---

def ros_callback_thread(msg):
    """
    Callback function that continuously updates Lidar data from ROS.
    Runs independently of the main loop, so data flow is never interrupted.
    """
    global scan_data
    with thread_lock:
        scan_data = msg

def navigation_control_thread():
    """
    [THE BRAIN]: Thread that makes robot movement decisions.
    Activates when user presses 'g', stops with 's'.
    Runs independently (Non-blocking) of terminal commands.
    """
    global is_navigating, system_active, scan_data
    
    rate = rospy.Rate(10) # 10 Hz (10 decisions per second)
    
    print(">>> Navigation Control Module Loaded. (Standing by...)")
    
    while system_active and not rospy.is_shutdown():
        # Wait if navigation is not active or data hasn't arrived
        if not is_navigating or scan_data is None:
            time.sleep(0.1)
            continue
            
        # Copy global data (For thread safety)
        with thread_lock:
            current_scan = get_clean_scan_data(scan_data)
        
        # Front Safety Distance (20-degree angle directly in front of robot)
        front_distance = min(current_scan[0:10] + current_scan[350:360])
        
        linear_x = 0.0
        angular_z = 0.0
        log_status = ""
        
        # --- FINITE STATE MACHINE ---
        
        # STATE 1: EMERGENCY BRAKE (Critical Distance)
        if front_distance < MIN_STOP_DIST:
            log_status = "[STATUS]: EMERGENCY STATE (Reverse Escape)"
            linear_x = -0.10 # Slowly reverse
            angular_z = 0.0
            
        # STATE 2: GAP ANALYSIS AND MOVEMENT
        else:
            target_angle, target_dist = find_best_gap(current_scan)
            
            if target_angle is None:
                # No place to go (Cornered)
                log_status = "[STATUS]: STUCK (Rotating)"
                linear_x = 0.0
                angular_z = 0.6 # Rotate in place
            else:
                # Calculate dynamic velocity
                linear_x = calculate_dynamic_velocity(front_distance)
                
                # Proportional (P-Control) rotation towards target angle
                angular_z = target_angle * 1.5 
                
                # Reduce linear speed during sharp turns to prevent drifting
                if abs(angular_z) > 0.5:
                    linear_x *= 0.5
                
                degree_display = math.degrees(target_angle)
                log_status = f"[STATUS]: CRUISING (Target: {degree_display:.1f} deg)"

        # Send calculated velocities to robot
        move.linear.x = linear_x
        move.angular.z = angular_z
        pub.publish(move)
        
        # Print status to terminal (Flows continuously, doesn't erase previous line)
        print(f"{log_status} | Speed: {linear_x:.2f} m/s | Front Dist: {front_distance:.2f}m")
        
        rate.sleep()


def stop_robot_emergency():
    """Safety function to stop the robot immediately."""
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    pub.publish(move) # Send twice to ensure delivery
    print("\n!!! ROBOT EMERGENCY STOPPED !!!\n")


# --- MAIN PROGRAM ---

if __name__ == '__main__':
    try:
        # 1. Start ROS Lidar Listener Thread
        # This thread continuously updates Lidar data in the background.
        ros_thread = threading.Thread(target=rospy.Subscriber, args=('/scan', LaserScan, ros_callback_thread))
        ros_thread.daemon = True # Closes when main program closes
        ros_thread.start()
        
        # 2. Start Navigation Logic Thread
        # This thread manages autonomous movements of the robot.
        nav_thread = threading.Thread(target=navigation_control_thread)
        nav_thread.daemon = True
        nav_thread.start()
        
        # 3. User Interface (Main Thread)
        # This only waits for user command, never freezes.
        print("==================================================")
        print("        OBSTACLE AVOIDANCE ROBOT DESIGN           ")
        print("==================================================")
        print("          AUTONOMOUS NAVIGATION SYSTEM            ")
        print("     [ MULTITHREADED & REACTIVE CONTROL ]         ")
        print("==================================================")
        print(" COMMAND LIST:")
        print("  g  -> GO    (Activate navigation)")
        print("  s  -> STOP  (Stop robot immediately)")
        print("  e  -> EXIT  (Close program and terminal)")
        print("==================================================")
        
        while not rospy.is_shutdown():
            # input() function blocks the main thread but other threads continue running.
            # This allows you to type here even while the robot is moving.
            user_input = input("\nEnter Command (g/s/e) > ").strip().lower()
            
            if user_input == 'g':
                if not is_navigating:
                    print(">>> SYSTEM STARTING...")
                    is_navigating = True
                else:
                    print(">>> WARNING: System is already running!")
                    
            elif user_input == 's':
                if is_navigating:
                    print(">>> STOP COMMAND RECEIVED!")
                    is_navigating = False
                    stop_robot_emergency()
                else:
                    print(">>> Robot is already stopped.")
            
            elif user_input == 'e':
                print(">>> EXITING...")
                is_navigating = False
                system_active = False # Terminate other threads
                stop_robot_emergency()
                time.sleep(1) # Wait for threads to close
                sys.exit() # Exit program
                
            else:
                print(">>> ERROR: Invalid command! Please enter 'g', 's' or 'e'.")
                
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        stop_robot_emergency()
        sys.exit()