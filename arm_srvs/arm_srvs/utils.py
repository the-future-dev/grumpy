import numpy as np
from geometry_msgs.msg import Pose

######################################
# This file contains the:
# Common functions for the arm services
# Common constants for the arm
######################################

# Origin of servo 5 in base_link frame:
x_origin_servo5 = -0.00450
y_origin_servo5 = -0.04750
z_origin_servo5 =  0.12915

z_oc_g = -0.01  # Distance from object centers to ground

# Angles of the servos for different tasks:
theta_servo5_pick = 60
theta_servo5_view = 30
theta_servo4_view = 60
theta_servo3_view = 90

# Constants in the robot arm links:
l_5_4     = 0.10048  # From joint of servo 5 to joint of servo 4:
l_4_3     = 0.094714  # From joint of servo 4 to joint of servo 3:
l_3_2_ee  = 0.05071 + 0.11260 # From joint of servo 3 to joint of servo 2 + from joint servo 2 to end effector
l_3_cam_z = 0.046483  # z-distance from joint of servo 3 to the arm camera
l_3_cam_x = 0.042169  # x-distance from joint of servo 3 to the arm camera

# Origin of servo 4 in rho+z-plane
z_origin_servo4   = z_origin_servo5 + l_5_4 * np.sin(np.deg2rad(90) - np.deg2rad(theta_servo5_pick))
rho_origin_servo4 = l_5_4 * np.cos(np.deg2rad(90) - np.deg2rad(theta_servo5_pick))
    
# Angles of the servos for different tasks:
initial_thetas = [3000, 12000, 12000, 12000, 12000, 12000]  # Arm pointing straight up, used for reset and driving around without object
drive_thetas   = [-1, 12000, 7000, 12000, 12000, 12000]  # Arm pointing straight up, gripper tilted forward, used for driving around with object
drop_thetas    = [-1 , -1, 3000, 14500, 9000, -1]  # Angles for droping objects into the bins
view_thetas    = [-1, -1, 3000, 18000, 9500, -1]  # Angles when the arm camera has a view over the entire pick-up area
still_thetas   = [-1] * 6 # Angles for when the arm should not move

times = [1500] * 6  # Standard angle movement times to all servos

servos_offset = 350 # Allowed offset for the servos to be considered at the correct position

# The position of the camera in the base_link frame when in view position
cam_pos = Pose()
cam_pos.position.x = x_origin_servo5 + l_5_4 * np.sin(np.deg2rad(theta_servo5_view)) + l_4_3 + l_3_cam_x
cam_pos.position.y = y_origin_servo5
cam_pos.position.z = z_origin_servo5 + l_5_4 * np.cos(np.deg2rad(theta_servo5_view)) - l_3_cam_z

# Camera parameters for calibration and undistortion of the image
intrinsic_mtx = np.array([[438.783367, 0.000000, 305.593336],
                          [0.000000, 437.302876, 243.738352],
                          [0.000000, 0.000000, 1.000000]])
dist_coeffs   = np.array([-0.361976, 0.110510, 0.001014, 0.000505, 0.000000])


def extract_object_position(node, pose:Pose):
    """
    Args:
        msg: Pose, required, the position and orientation of the object
    Returns:
        x: float, x-position of the object in base_link frame
        y: float, y-position of the object in base_link frame
        z: float, z-position of the object in base_link frame
    Other functions:

    """

    assert isinstance(pose, Pose), node._logger.error(f'request was not type Pose')  # Assert that the request has the correct type

    x, y, z = pose.position.x, pose.position.y, pose.position.z

    assert isinstance(x, float), node._logger.error('x was not type float')
    assert isinstance(x, float), node._logger.error('y was not type float')
    assert isinstance(x, float), node._logger.error('z was not type float')

    node._logger.info('Got the position of the object')

    return x, y, z


def get_theta_6(x, y):
    """
    Args:
        x: float, required, x-position of the object in base_link frame
        y: float, required, y-position of the object in base_link frame
    Returns:
        delta_theta_6: float, degrees that servo 6 has to rotate from its position
    Other functions:

    """

    x_dist        = x - x_origin_servo5  # The distance from the origin of servo 5 to the object in the x direction
    y_dist        = y - y_origin_servo5  # The distance from the origin of servo 5 to the object in the y direction
    delta_theta_6 = np.rad2deg(np.arctan2(y_dist, x_dist))  # Calculate the angle for servo 6 in radians and convert to degrees

    return round(initial_thetas[5] + delta_theta_6 * 100)  # New angle of servo 6, round and convert to int


def check_angles_and_times(node, angles, times):
    """
    Args:
        angles: list, required, the angles for each servo to be set to
        times : list, required, the times for each servo to get to the given angle
    Returns:
        
    Other functions:
        Raises error if the angles and times are not in the correct format, length or interval
    """

    assert isinstance(angles, list), node._logger.error('angles is not of type list')
    assert isinstance(times, list), node._logger.error('times is not of type list')
    assert len(angles) == 6, node._logger.error('angles was not of length 6')
    assert len(times) == 6, node._logger.error('times was not of length 6')
    assert all(isinstance(angle, int) for angle in angles), node._logger.error('angles was not of type int')
    assert all(isinstance(time, int) for time in times), node._logger.error('times was not of type int')
    assert all(1000 <= time <= 5000 for time in times), node._logger.error('times was not within the interval [1000, 5000]')
    assert (0 <= angles[0] <= 13000) or (angles[0] == -1), node._logger.error(f'servo 1 was not within the interval [0, 11000] or -1, got {angles[0]}')
    assert (0 <= angles[1] <= 24000) or (angles[1] == -1), node._logger.error(f'servo 2 was not within the interval [0, 24000] or -1, got {angles[1]}')
    assert (2500 <= angles[2] <= 21000) or (angles[2] == -1), node._logger.error(f'servo 3 was not within the interval [2500, 21000] or -1, got {angles[2]}')
    assert (3000 <= angles[3] <= 21500) or (angles[3] == -1), node._logger.error(f'servo 4 was not within the interval [3000, 21500] or -1, got {angles[3]}')
    assert (6000 <= angles[4] <= 18000) or (angles[4] == -1), node._logger.error(f'servo 5 was not within the interval [6000, 18000] or -1, got {angles[4]}')
    assert (0 <= angles[5] <= 20000) or (angles[5] == -1), node._logger.error(f'servo 6 was not within the interval [0, 20000] or -1, got {angles[5]}')

    node._logger.info('Checked the angles and times')


def inverse_kinematics(x, y, z):
    """
    Args:
        x: float, required, x-position of the object in base_link frame
        y: float, required, y-position of the object in base_link frame
        z: float, required, z-position of the object in base_link frame
    Returns:
        delta_theta_3: float, degrees that servo 3 has to rotate from its position
        delta_theta_4: float, degrees that servo 4 has to rotate from its position
    Other functions:

    """
    # The hypotenuse (rho) from the origin of servo 5 to the object position in the xy-plane minus the distance servo 4 has already moved
    rho_dist = (np.sqrt((x - x_origin_servo5) ** 2 + (y - y_origin_servo5) ** 2) - rho_origin_servo4) * 1.03
    # z_dist   = utils.z_oc_g - utils.z_origin_servo4  # The combined distance to the grip point in the z direction
    z_dist   = z - z_origin_servo4  # The combined distance to the grip point in the z direction

    # Calculate the angles for servo 3 and 4 in radians
    # cos_d_t_servo3     = (rho_dist ** 2 + z_dist ** 2 - l_4_3 ** 2 - l_3_2_ee ** 2) / (2 * l_4_3 * l_3_2_ee)
    # delta_theta_servo3 = - np.arctan2(np.sqrt(1 - cos_d_t_servo3 ** 2), cos_d_t_servo3)
    delta_theta_servo3 = - np.arccos((rho_dist ** 2 + z_dist ** 2 - l_4_3 ** 2 - l_3_2_ee ** 2) /  (2 * l_4_3 * l_3_2_ee))
    delta_theta_servo4 = (np.arctan2(z_dist, rho_dist) - 
                          np.arctan2(l_3_2_ee * np.sin(delta_theta_servo3), l_4_3 + (l_3_2_ee * np.cos(delta_theta_servo3))))
    
    # Convert the angles to degrees and adjust for the initial angle of servo 5
    delta_theta_servo3 = np.rad2deg(delta_theta_servo3)
    delta_theta_servo4 = (- np.rad2deg(delta_theta_servo4)) + (90 - theta_servo5_pick)

    theta_servo3 = round(initial_thetas[2] + delta_theta_servo3 * 100)  # New angle of servo 3, round and convert to int
    theta_servo4 = round(initial_thetas[3] + delta_theta_servo4 * 100)  # New angle of servo 4, round and convert to int

    return theta_servo3, theta_servo4


def changed_thetas_correctly(pub_angles, curr_angles):
    """
    Args:
        pub_angles : list, required, the angles that were published to the arm
        curr_angles: list, required, the angles that the arm is at after the angles have been published and the required time has passed
    Returns:
        bool, if the arm has moved to the published angles, except for the first servo as it is not reliable
    Other functions:
        
    """

    correct = True

    if len(pub_angles) != len(curr_angles):
        correct = False
    
    for i in range(1, len(pub_angles)):
        if pub_angles[i] == -1:
            continue
        elif not np.isclose(curr_angles[i], pub_angles[i], atol=servos_offset):
            correct = False
              
    return correct