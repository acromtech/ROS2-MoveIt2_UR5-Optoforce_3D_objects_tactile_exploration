"""
Simulation class for robotics simulation using PyBullet.

PyBullet is a fast and easy-to-use Python module for robotics simulation and machine learning.

Let’s see how to install pybullet on your Windows PC:

1. Verifying Python installation:
   - Open your command-line tool.
   - Run the command:
       py --version
   - If Python is installed, you should get output like "Python 3.x", where the number is your current Python version.
   - If you do not have Python, please install the latest 3.x version (2021) from python.org.

2. Making sure you can run pip commands:
   - Run the command:
       py -m pip --version
   - If it doesn’t return a version number of your installed pip, install it following any tutorial from the internet.

3. Installing Microsoft Visual Studio Build Tools:
   - Go to the download page for Microsoft Visual Studio Build Tools and download Build Tools for the last Visual Studio version 
        https://aka.ms/vs/17/release/vs_BuildTools.exe (click here to download the installer automaticly)
   - Open the installer and make sure "Desktop development with C++" is selected.
   - Click on the download/install/modify button.

If you have any problem check the following page : https://deepakjogi.medium.com/how-to-install-pybullet-physics-simulation-in-windows-e1f16baa26f6

4. When it's fully installed, uncomment the lines below to install pybullet:
import os
os.system("pip3 install --upgrade setuptools")
os.system("pip3 install pybullet")

When you successfully installed pybullet you can use the Simulation class as you want :
-> Firstly you can put "sim=Simulation()" in your main to launch a test code
You can check the pybullet's Github to learn more about the library : https://github.com/bulletphysics/bullet3?tab=readme-ov-file
"""
import os
import time
#os.system('pip3 install pybullet')
import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

PRISMATIC = 1
REVOLUTE = 0
class Simulation:
    def __init__(self, urdf_file=None, startPos=[0,0,1], startOrientation=[0,0,0],fixedBase=False, viewMode=False):
        """
        Initialize the simulation with a URDF file.

        Parameters:
            urdf_file (str): Path to the URDF file.
            startPos (list, optional): Starting position of the robot. Default is [0,0,1].
            startOrientation (list, optional): Starting orientation of the robot. Default is [0,0,0].
            fixedBase ...
        """
        # Initialize parameters
        self.urdf_file=urdf_file
        self.startPos=startPos
        self.startOrientation=p.getQuaternionFromEuler(startOrientation)
        self.fixedBase=fixedBase

        # Launch pybullet and import the simulation plane and the input urdf file
        self.physicsClient = p.connect(p.GUI)
        p.setRealTimeSimulation(1, self.physicsClient)
        #p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
        
        print(pybullet_data.getDataPath())
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.planeId = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=self.fixedBase)
        
        path = os.getcwd()+"/mechanics/urdf"
        path = path.replace("\\", "/")
        print(path)
        p.setAdditionalSearchPath(path)
        self.robot_id = p.loadURDF(f"{urdf_file}", self.startPos, self.startOrientation, useFixedBase=self.fixedBase)

        self.numJoint = p.getNumJoints(self.robot_id)
        self.endEffectorIndex = self.numJoint-1
        self.jointName = []
        self.jointType = []
        self.jointLowerLimit = []
        self.jointUpperLimit = []
        self.jointMaxForce = []
        self.jointMaxVelocity = []
        self.jointFramePosition = []
        self.jointFrameOrientation = []

        self.endEffector_offset = [0.0, 0.0, 0.0]
        self.endEffector_frame = self.get_robot_current_position()

        for i in range(self.numJoint):
            self.joint_info = p.getJointInfo(self.robot_id, i)
            self.jointName.append(self.joint_info[1].decode('utf-8'))
            self.jointType.append(self.joint_info[2])
            self.jointLowerLimit.append(self.joint_info[8])
            self.jointUpperLimit.append(self.joint_info[9])
            self.jointMaxForce.append(self.joint_info[10])
            self.jointMaxVelocity.append(self.joint_info[11])
            self.jointFramePosition.append(self.joint_info[14])
            self.jointFrameOrientation.append(self.joint_info[15])

            print(f"Joint Frame{i} Position:    \t",self.jointFramePosition[i])
            print(f"Joint Frame{i} Orientation: \t",self.jointFrameOrientation[i])
        
        print("NumJoint:                \t",self.numJoint)
        print("EndEffectorIndex:        \t",self.endEffectorIndex)
        print("Names:                   \t",self.jointName)
        print("Types:                   \t",self.jointType)
        print("Lower Limits:            \t",self.jointLowerLimit)
        print("Upper Limits:            \t",self.jointUpperLimit)
        print("Max Force:               \t",self.jointLowerLimit)
        print("Max Velocity:            \t",self.jointUpperLimit)
        print("Start Position:          \t",self.startPos)
        print("Start Orientation:       \t",self.startOrientation)

    def view_mode(self):
        print("------ Press ENTER to quit ------")
        if input(): pass
    
    # ------------ MOVE ROBOT JOINTS ------------

    def move_joint(self, joint_index, displacement, max_speed=1, torque=1000., acceptable_error=1E-5, wait_to_exit=False, plot_joint_positions=False):
        """
        Move a joint of the robot to the target position.

        Parameters:
            joint_index (int): Index of the joint to move.
            displacement        (float)     distance in meter if it's a PRISMATIC joint 
                                            angle in degree if it's a REVOLUTE joint
            max_speed           (float)     Speed at which to move the joint.       Default is 1.
            torque              (float)     Torque at which to move the joint.      Default is 10.
            positionGain        (float)     kp position gain of the joint           Default is 0.5
            velocityGain        (float)     kd velocity gain of the joint           Default is 0.225
        """
        if self.get_joint_angle_current_position(joint_index) != displacement:
            p.setJointMotorControl2(bodyIndex           =self.robot_id,
                                    jointIndex          =joint_index,
                                    controlMode         =p.POSITION_CONTROL,
                                    targetPosition      =displacement,
                                    maxVelocity         =max_speed,
                                    force               =torque)
            if wait_to_exit:
                joint_traj = [[] for _ in range(1)]
                start_time = time.time()
                while True :
                    joint_current_position = self.get_joint_angle_current_position(joint_index)
                    current_time = time.time() - start_time
                    joint_traj[0].append((joint_current_position, current_time))
                    if abs(self.get_joint_angle_current_position(joint_index) - displacement) < acceptable_error: break
                if plot_joint_positions: self.plot_joint_positions(joint_traj, [displacement])


    def move_joints(self, joint_indices, displacements, max_speeds=None, torques=None, acceptable_error=1E-5, wait_to_exit=False):
        """
        Move joints of the robot to the target positions.

        Parameters:
            joint_indices (list of int): Indices of the joints to move.
            displacements (list of float): Distances in meters if they are PRISMATIC joints, or angles in degrees if they are REVOLUTE joints.
            max_speeds (list of float): Speeds at which to move the joints. Default is [1.].
            torques (list of float): Torques at which to move the joints. Default is [10.].
            positionGains (list of float): kp position gains of the joints. Default is [0.5].
            velocityGains (list of float): kd velocity gains of the joints. Default is [0.225].
            wait_to_exit (bool): Whether to wait for the joints to reach their target positions before returning. Default is False.
        """
        if max_speeds==None: max_speeds=[1]*self.get_num_joint()
        if torques==None: torques=[100]*self.get_num_joint()

        p.setJointMotorControlArray(self.robot_id,joint_indices,p.POSITION_CONTROL,displacements,max_speeds,torques)

        if wait_to_exit:
            while True:
                positions = [self.get_joint_angle_current_position(joint_index) for joint_index in joint_indices]
                if all(abs(position - target) < acceptable_error for position, target in zip(positions, displacements)):
                    break
    
    # ------------ MODEL CALCULATIONS ------------

    def calculate_inverse_kinematics(self, targetPosition):
        return p.calculateInverseKinematics(bodyUniqueId           =self.robot_id,
                                            endEffectorLinkIndex   =self.endEffectorIndex,
                                            targetPosition         =targetPosition)
    
    def calculate_inverse_dynamics(self, jointPositions=[], jointVelocities=[], jointAccelerations=[]):
        return p.calculateInverseDynamics(bodyUniqueId          =self.robot_id,
                                            objPositions        =jointPositions,
                                            objVelocities       =jointVelocities,
                                            objAccelerations    =jointAccelerations,
                                            physicsClientId     =self.physicsClient)
    
    # ------------ GET JOINTS INFO ------------

    def get_num_joint(self):
        return self.numJoint
    
    def get_type_joint(self, indice):
        return self.jointType[indice]
    
    def get_joint_angle_current_position(self,joint_indice):
        return p.getJointState(self.robot_id,joint_indice)[0]

    def get_joint_angle_current_positions(self):
        return [self.get_joint_angle_current_position(i) for i in range(self.get_num_joint())]
    
    def get_joint_frame_current_position(self, joint_index):
        return p.getJointInfo(self.robot_id, joint_index)[14]
    
    def get_joint_frame_current_positions(self):
        return [self.get_joint_frame_current_position(i) for i in range(self.get_num_joint())]
    
    # ------------ GET ROBOT INFO ------------
    
    def get_robot_current_position(self):
        data = p.getLinkState(bodyUniqueId=self.robot_id, linkIndex=self.endEffectorIndex, computeForwardKinematics=True)
        linkWorldPosition=data[0]
        linkWorldOrientation=data[1]
        worldLinkFramePosition=data[4]
        worldLinkFrameOrientation=data[5]
        return [linkWorldPosition, linkWorldOrientation, worldLinkFramePosition, worldLinkFrameOrientation]
        #parent_frame_position = p.getJointInfo(self.robot_id, self.get_num_joint()-1)[14]
        #endEffector_position = [parent_frame_position[i] + self.endEffector_offset[i] for i in range(3)]
        #return endEffector_position
    
    def get_robot_current_position_all_link_frames(self):
        data = []
        linkWorldPosition, linkWorldOrientation, worldLinkFramePosition, worldLinkFrameOrientation = [], [], [], []
        for i in range(self.endEffectorIndex):
            data = p.getLinkState(bodyUniqueId=self.robot_id, linkIndex=i, computeForwardKinematics=True)
            linkWorldPosition.append(data[0])
            linkWorldOrientation.append(data[1])
            worldLinkFramePosition.append(data[4])
            worldLinkFrameOrientation.append(data[5])
        return [linkWorldPosition, linkWorldOrientation, worldLinkFramePosition, worldLinkFrameOrientation]
    
    # ------------ OFFSET ------------

    def set_endEffector_frame_offset(self, X, Y, Z):
        self.endEffector_offset = [X, Y, Z]
        endEffector_data = self.get_robot_current_position()
        self.endEffector_frame = [endEffector_data[2][0] + X, endEffector_data[2][1] + Y, endEffector_data[2][2] + Z]
        # self.constraint_id = p.createConstraint(parentBodyUniqueId       =self.robot_id,
        #                                     parentLinkIndex         =self.get_num_joint()-1,
        #                                     childBodyUniqueId       =-1, # Aucun corps enfant (par rapport au monde)
        #                                     childLinkIndex          =-1, # Aucun lien enfant (par rapport au monde)
        #                                     jointType               =p.JOINT_FIXED, # Garder le joint fixe pour appliquer un décalage
        #                                     jointAxis               =[0, 0, 0], # Aucun axe de rotation
        #                                     parentFramePosition     =endEffector_data[2], # Avant d'appliquer l'offset
        #                                     childFramePosition      =self.endEffector_frame)  # Après application de l'offset
    
    def get_endEffector_frame(self):
        return self.endEffector_frame

    def get_endEffector_offset(self):
        return self.endEffector_offset

    # ------------ VIEW FRAMES ------------

    def update_frame_axes(self,frame_position, axis_length=0.5):
        p.addUserDebugLine(frame_position, [frame_position[0] + axis_length, frame_position[1], frame_position[2]], [1, 0, 0], 3, 0)
        p.addUserDebugLine(frame_position, [frame_position[0], frame_position[1] + axis_length, frame_position[2]], [0, 1, 0], 3, 0)
        p.addUserDebugLine(frame_position, [frame_position[0], frame_position[1], frame_position[2] + axis_length], [0, 0, 1], 3, 0)

    def delete_frames_axes(self):
        p.removeAllUserDebugItems()

    def view_link_frame(self, link_index, print_terminal=True):
        frame = self.get_robot_current_position_all_link_frames()[2][link_index]
        self.update_frame_axes(frame)
        if print_terminal: print(f"Link frame {link_index}:                   ",frame)

    def view_all_link_frames(self, print_terminal=True):
        for i in range(self.endEffectorIndex):
            self.view_link_frame(i, print_terminal)

    def view_joint_frame(self, joint_index, print_terminal=True):
        frame = self.get_joint_frame_current_position(joint_index)
        self.update_frame_axes(frame)
        if print_terminal: print(f"Joint frame {joint_index}:                  ",frame)
    
    def view_all_joint_frames(self, print_terminal=True):
        for i in range(self.get_num_joint()):
            self.view_joint_frame(i, print_terminal)

    def view_endEffector_frame(self, print_terminal=True):
        self.update_frame_axes(self.endEffector_frame)
        if print_terminal: print(f"End effector frame:                         ",self.endEffector_frame)

    # ------------ GENERATE GRAPHS ------------
    
    def plot_joint_positions(self, joint_traj, target_q):
        plt.figure(figsize=(10, 6))
        for i, traj in enumerate(joint_traj):
            timestamps = [entry[1] for entry in traj]
            positions = [entry[0] for entry in traj]
            plt.plot(timestamps, positions, label=f"Joint {i}")
            plt.plot([timestamps[0], timestamps[-1]], [target_q[i], target_q[i]], label=f"Target Position Joint {i}", linestyle='--', color=plt.gca().lines[-1].get_color())
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Position')
        plt.title('Joint Positions Over Time')
        plt.legend()
        plt.show()

    def get_end_effector_forces(self):
        """
        Retrieve force and torque data from the end effector.
        """
        joint_states = p.getJointStates(self.robot_id, range(self.numJoint))
        torques = [state[3] for state in joint_states]  # Retrieve torque information from joint states
        return torques
    
    def get_contact_forces(self):
        """
        Retrieve contact forces applied to the end effector.
        """
        contacts = p.getContactPoints(bodyA=self.robot_id)
        forces = []
        for contact in contacts:
            forces.append(contact[9])  # Normal force
        return forces

    def print_end_effector_forces(self):
        torques = self.get_end_effector_forces()
        print(f"Torques at joints: {torques}")
        forces = self.get_contact_forces()
        print(f"Contact forces: {forces}")

    def get_contact_info(self):
        contact_points = p.getContactPoints(bodyA=self.robot_id)
        contact_info = []
        for point in contact_points:
            info = {
                "contact_position_on_a": point[5],
                "contact_position_on_b": point[6],
                "contact_normal_on_b": point[7],
                "contact_distance": point[8],
                "normal_force": point[9],
                "lateral_friction_force_1": point[10],
                "lateral_friction_force_2": point[12],
            }
            contact_info.append(info)
        return contact_info
    
    def add_cube(self, position, size=1.0):
        """
        Add a cube to the simulation.

        Parameters:
            position (list): [x, y, z] coordinates of the cube's position.
            size (float, optional): Size of the cube. Default is 1.0.
        """
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[size/2, size/2, size/2])
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[size/2, size/2, size/2])
        cube_id = p.createMultiBody(baseMass=1,
                                    baseInertialFramePosition=[0, 0, 0],
                                    baseCollisionShapeIndex=collision_shape_id,
                                    baseVisualShapeIndex=visual_shape_id,
                                    basePosition=position)
        return cube_id

    def add_sphere(self, position, radius=0.5):
        """
        Add a sphere to the simulation.

        Parameters:
            position (list): [x, y, z] coordinates of the sphere's position.
            radius (float, optional): Radius of the sphere. Default is 0.5.
        """
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=radius)
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=radius)
        sphere_id = p.createMultiBody(baseMass=1,
                                      baseInertialFramePosition=[0, 0, 0],
                                      baseCollisionShapeIndex=collision_shape_id,
                                      baseVisualShapeIndex=visual_shape_id,
                                      basePosition=position)
        return sphere_id

    def add_cylinder(self, position, radius=0.5, height=1.0):
        """
        Add a cylinder to the simulation.

        Parameters:
            position (list): [x, y, z] coordinates of the cylinder's position.
            radius (float, optional): Radius of the cylinder. Default is 0.5.
            height (float, optional): Height of the cylinder. Default is 1.0.
        """
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_CYLINDER, radius=radius, length=height)
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_CYLINDER, radius=radius, height=height)
        cylinder_id = p.createMultiBody(baseMass=1,
                                        baseInertialFramePosition=[0, 0, 0],
                                        baseCollisionShapeIndex=collision_shape_id,
                                        baseVisualShapeIndex=visual_shape_id,
                                        basePosition=position)
        return cylinder_id


# Initialize the simulation
sim = Simulation("UR5_simplified/urdf/UR5_simplified.urdf", startPos=[0,0,0], fixedBase=True, viewMode=False)

sim.view_mode()