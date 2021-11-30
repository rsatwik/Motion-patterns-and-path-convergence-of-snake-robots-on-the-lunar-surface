#!/usr/bin/env python
"""
The node publishes joint position commands to effort position controllers.
The controllers should already be spawned and running and communicating
with the appropriate robot hardware interface.
"""

import rospy
from std_msgs.msg import Float64

import numpy as np

class JointCmds:
    """
    The class provides a dictionary mapping joints to command values.
    """
    def __init__( self, num_mods ) :
        
        self.num_modules = num_mods
        self.jnt_cmd_dict = {}        
        self.joints_list = []
        self.t = 0.0
        
        for i in range(self.num_modules) :
            leg_str='S_'
            if i < 10 :
                leg_str += '0' + str(i)
            else :
                leg_str += str(i)
            self.joints_list += [leg_str]

    def update( self, dt ) :

        self.t += dt

        ## sidewinding gait ##
        # spatial frequency (2pi/12)
        #spat_freq = 0.08
          
        # temporal phase offset between horizontal and vertical waves
        #TPO = 0.375

        # amplitude
       

        # direction
        # d = -1

        # if even
            # command = A*sin( 2.0*np.pi*(d*t + module_index*spat_freq) )
        # if odd
            # command = A*sin( 2.0*np.pi*(d*t + TPO + module_index*spat_freq) )

        for i, jnt in enumerate(self.joints_list):
            if(i%2) == 0:
                self.jnt_cmd_dict[jnt] = 0*np.sin(((i*2*np.pi)/4) - (1.9*np.pi*self.t) + 0*np.pi/2 ) #even
            else:
                self.jnt_cmd_dict[jnt] = np.sin(((i*8*np.pi)/12)  - (2*np.pi*self.t))  #odd
                 
        return self.jnt_cmd_dict


def publish_commands( num_modules, hz ):
    pub={}
    ns_str = '/snake'
    cont_str = 'eff_pos_controller'
    for i in range(num_modules) :
        leg_str='S_'
        if i < 10 :
            leg_str += '0' + str(i)
        else :
            leg_str += str(i)
        pub[leg_str] = rospy.Publisher( ns_str + '/' + leg_str + '_'
                                        + cont_str + '/command',
                                        Float64, queue_size=10 )
    rospy.init_node('snake_controller', anonymous=True)
    rate = rospy.Rate(hz)
    jntcmds = JointCmds(num_mods=num_modules)
    while not rospy.is_shutdown():
        jnt_cmd_dict = jntcmds.update(1./hz)
        for jnt in jnt_cmd_dict.keys() :
            pub[jnt].publish( jnt_cmd_dict[jnt] )
        rate.sleep()


if __name__ == "__main__":
    try:
        num_modules = 16        
        hz = 100
        publish_commands( num_modules, hz )
    except rospy.ROSInterruptException:
        pass
