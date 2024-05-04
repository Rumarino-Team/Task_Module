import os
from controller import PIDRegulator
from Task_Module import PIDParams
from controller import thruster
import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from data import initialize_data
class ThrusterAllocator:
    DEFAULT_AXIS = np.array([1, 0, 0, 0])  # Default thrust direction

    def __init__(self, t_1, t_2, positions, orientations, axes=None, pid_params=None):

        rospy.init_node('thruster_allocator')
        initialize_subscriber()
        shared_data = initialize_data()


    
        self.s = rospy.Service('set_pid_parameters', PIDParams, self.set_pid_parameters)
        self.thrusters_pubishers = []

        for i in range(1,9):
            self.thrusters_publishers.append(rospy.Publisher('/thrusters/' + i + '/', Vector3, queue_size=10))
        rate = rospy.Rate(10)
        
        # Set your constant number of controllers in this allocator
        # This means you control the number of controllers you want 
        # to use.



        if axes is None:
            axes = [self.DEFAULT_AXIS] * len(positions)

        self.controller_depth = PIDRegulator(1, 0, 0, 1)
        self.controller_surge = PIDRegulator(1, 0, 0, 1)
        self.controller_sway = PIDRegulator(1, 0, 0, 1)
        self.controller_yaw = PIDRegulator(1, 0, 0, 1)


        # Normally we dont use multiple allocation matrices
        # but in this case because we want to have two controllers
        # that act at the same time with unshared thrusters we are
        # going to use this method.
        self.allocation_matrix_collection_1 = np.zeros(t_1, 3)
        self.allocation_matrix_collection_2 = np.zeros(t_2, 1)
        
        while not rospy.is_shutdown():
            # Get the desired forces from the controllers
            gen_forces = np.array([self.controller_depth.regulate(), self.controller_surge.regulate(), self.controller_yaw.regulate()])
            # Compute the thruster forces
            thrust = self.compute_thruster_forces(gen_forces)
            # Publish the thruster forces
            for i in range(8):
                self.thrusters[i].publish(Vector3(thrust[i], thrust[i], thrust[i]))
            rate.sleep()
    def compute_configuration_matrix(self, positions, orientations, axes):
        num_thrusters = len(positions)
        force_dist_matrix = []

        for i in range(num_thrusters):
            thrust_body = transformations.quaternion_matrix(orientations[i]).dot(
                axes[i].transpose())[0:3]
            torque_body = np.cross(positions[i], thrust_body)
            # Append the force and torque vectors for each thruster to the matrix
            force_dist_matrix.append(np.hstack((thrust_body, torque_body)))
        
        # Stack all rows to form the configuration matrix
        return np.vstack(force_dist_matrix).T  # Transpose if necessary to match the expected dimensionality


        
    def set_pid_parameters(self, depth_pid, surge_pid, yaw_pid):
        self.controller_depth.set_parameters(depth_pid[0], depth_pid[1], depth_pid[2])
        self.controller_surge.set_parameters(surge_pid[0], surge_pid[1], surge_pid[2])
        self.controller_yaw.set_parameters(yaw_pid[0], yaw_pid[1], yaw_pid[2])

    def compute_thruster_forces(self, gen_forces):
            """Compute desired thruster forces using the inverse configuration
            matrix.
            """
            # Calculate individual thrust forces
            thrust = self.inverse_configuration_matrix.dot(gen_forces)
            for i in range(self.n_thrusters):
                if abs(thrust[i]) > max_thrust[i]:
                    thrust[i] = np.sign(thrust[i]) * max_thrust[i]
            return thrust

if __name__ == '__main__':
    # inittialize the thruster allocator
    try:
        thruster_allocator = ThrusterAllocator(4, 4)
    except rospy.ROSInterruptException:
        pass
    