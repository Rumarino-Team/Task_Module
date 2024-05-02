import os
from controller import PIDRegulator
from Task_Module import PIDParams
import numpy as np
import rospy
from geometry_msgs.msg import Vector3

class ThrusterAllocator:
    def __init__(self, t_1, t_2):
        rospy.init_node('thruster_allocator')
    
        self.s = rospy.Service('set_pid_parameters', PIDParams, self.set_pid_parameters)
        self.thrusters = []
        for i in range(1,9):
            self.thrusters.append(rospy.Publisher('/thrusters/' + i + '/', Vector3, queue_size=10))
        rate = rospy.Rate(10)
        
        # Set your constant number of controllers in this allocator
        # This means you control the number of controllers you want 
        # to use.
        self.controller_depth = PIDRegulator(1, 0, 0, 1)
        self.controller_surge = PIDRegulator(1, 0, 0, 1)
        self.controller_yaw = PIDRegulator(1, 0, 0, 1)


        # Normally we dont use multiple allocation matrices
        # but in this case because we want to have two controllers
        # that act at the same time with unshared thrusters we are
        # going to use this method.
        allocation_matrix_collection_1 = np.zeros(t_1, 3)
        allocation_matrix_collection_2 = np.zeros(t_2, 3)
        
        while not rospy.is_shutdown():
            # Get the desired forces from the controllers
            gen_forces = np.array([self.controller_depth.regulate(), self.controller_surge.regulate(), self.controller_yaw.regulate()])
            # Compute the thruster forces
            thrust = self.compute_thruster_forces(gen_forces)
            # Publish the thruster forces
            for i in range(8):
                self.thrusters[i].publish(Vector3(thrust[i], thrust[i], thrust[i]))
            rate.sleep()
        
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
            # Obey limit on max thrust by applying a constant scaling factor to all
            # thrust forces
            limitation_factor = 1.0
            if type(self.config['max_thrust']) == list:
                if len(self.config['max_thrust']) != self.n_thrusters:
                    raise rospy.ROSException('max_thrust list must have the length'
                                            ' equal to the number of thrusters')
                max_thrust = self.config['max_thrust']
            else:
                max_thrust = [self.config['max_thrust'] for _ in range(self.n_thrusters)]
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
    