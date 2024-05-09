from Simulation import Simulation
import numpy as np
import time

sim = Simulation("UR5_simplified/urdf/UR5_simplified.urdf", startPos=[0,0,0], fixedBase=True, viewMode=False)
sim.calculate_inverse_kinematics([0,0,0])
for i in range (6):
    sim.move_joint(i,np.pi/2,1,1000)
    time.sleep(3)
    sim.move_joint(i,-np.pi/2,1,1000)
    time.sleep(3)



sim.view_mode()