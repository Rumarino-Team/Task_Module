from controller import PIDRegulator
import numpy as np
import rospy
class ThrusterAllocator:
    def __init__(self, t_1, t_2):
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
