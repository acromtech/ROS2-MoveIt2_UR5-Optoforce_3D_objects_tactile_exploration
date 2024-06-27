from Simulation import Simulation
import numpy as np
import time

# Initialize the simulation
sim = Simulation("UR5_simplified/urdf/UR5_simplified.urdf", startPos=[0,0,0], fixedBase=True, viewMode=False)
sim.calculate_inverse_kinematics([0,0,0])

# Add a cube, sphere, and cylinder to the simulation
cube_id = sim.add_cube(position=[0.5, 0.5, 0.5], size=0.2)
sphere_id = sim.add_sphere(position=[1, 0.5, 0.5], radius=0.1)
cylinder_id = sim.add_cylinder(position=[1.5, 0.5, 0.5], radius=0.1, height=0.3)

# Move the robot to interact with the objects
for i in range(6):
    sim.move_joint(i, np.pi/2, 1, 1000)
    time.sleep(3)
    sim.move_joint(i, -np.pi/2, 1, 1000)
    time.sleep(3)
    sim.print_end_effector_forces()
    contact_info = sim.get_contact_info()
    print(f"Contact info: {contact_info}")

sim.view_mode()
