



#  Edge cases are bool function that given the Shared Data will return True
# if a Edge case has been detected or False if everything is okay.
import numpy as np
import cv2


def is_close_to_plane(plane_coeffs, point, threshold):
    A, B, C, D = plane_coeffs
    x0, y0, z0 = point
    
    distance = abs(A*x0 + B*y0 + C*z0 + D) / np.sqrt(A**2 + B**2 + C**2)
    return distance <= threshold

def detect_color_wall(image, target_color_range, area_threshold):
    # Convert image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Create a mask for the target color
    mask = cv2.inRange(hsv_image, target_color_range[0], target_color_range[1])
    
    # Optionally, apply some morphological operations to clean up the mask
    
    # Check if the area of the detected color is significant
    if cv2.countNonZero(mask) / mask.size > area_threshold:
        return True
    else:
        return False


movement_edge_cases = [
    "on_the_surface_of_the_pool_and_waypoint_is_not_reachable",
    "very_close_to_the_wall",
    "not_enough_baterry_to_reach_waypoint",
    ""
]

DENSITY_WATER = 1000 # kg/m^3

def pressure_given_depth(depth):
    return 9.81 * DENSITY_WATER * depth


def movement_edge_case_callback(shared_data,waypoint, config_data):
    #  1) Check if the Submarine is on the surface of the pool
        #  Check the Depth Sensor to calculate if we are on the surface of the pool
        #  If Yes check if the Waypoint is above the surface. This means that the waypoint 
        # Must be deleted and we must continue with the next waypoint or task.
    
    # For now we can use the  DVL data and infered the pressured data

    if shared_data.dvl_data.depth >= config_data['pool_depth']: # Check if depth is on the limit of the pool surface. 
        if waypoint.point.z > config_data['pool_depth']: # Check if waypoint is above surface
            return True


    #  2) Check if the Submarine is very close to the wall
        #  Check the Image Data of the Camera. If there is a straight wall very close to the camera
        #  we must stop the movement and wait for the user to move the submarine away from the wall.
        # Probrably realizing a rotation of the submarine.
            
    if not len(shared_data.zed_data["objects_stamped"]) and shared_data.zed_data["plane_stamped"]:
        # Check if camera detects a plane very close
        plane_coeffs = shared_data.zed_data["plane_stamped"].plane_coefficients
        point = [shared_data.submarine_pose.position.x, shared_data.submarine_pose.position.y, shared_data.submarine_pose.position.z]

        if is_close_to_plane(plane_coeffs, point, 0.5):
            return True
        

    


    


    #  3) Check if the Submarine has enough battery to reach the waypoint
    #  Check the Battery Data of the Submarine. If the battery is less than 10% we must stop the movement


    #  4) Check if the Submarine movement is unstable
    #     If the submarine Controls parameters are not well defined we must stop the movement.
            
    


    



