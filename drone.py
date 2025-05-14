import pybullet as p
import numpy as np
from collections import deque

class Drone:
    def __init__(self, env, start_pos=[0, 0, 1]):
        self.env = env
        self.start_pos = start_pos
        self.current_pos = start_pos.copy()
        self.current_orientation = [0, 0, 0, 1]  # Quaternion
        
        # Drone parameters
        self.mass = 1.0
        self.max_thrust = 20.0
        self.battery = 100.0
        self.battery_drain_rate = 0.1
        self.max_speed = 5.0
        self.carrying_package = False
        self.current_package_id = None
        
        # Navigation
        self.target_pos = None
        self.path = deque()
        self.reached_target = True
        
        # PID controllers for each axis
        self.pid_x = PIDController(kp=1.0, ki=0.01, kd=0.1)
        self.pid_y = PIDController(kp=1.0, ki=0.01, kd=0.1)
        self.pid_z = PIDController(kp=2.0, ki=0.05, kd=0.2)
        
        self.setup_drone()
    
    def setup_drone(self):
        # Create drone body
        drone_size = [0.3, 0.3, 0.1]
        self.drone_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=drone_size)
        self.drone_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=drone_size, rgbaColor=[0,0,1,1])
        self.drone_id = p.createMultiBody(baseMass=self.mass, baseCollisionShapeIndex=self.drone_shape,
                                         baseVisualShapeIndex=self.drone_visual, basePosition=self.start_pos)
        
        # Create rotors (visual only)
        rotor_size = [0.1, 0.1, 0.02]
        rotor_color = [0.8, 0.8, 0.8, 1]
        rotor_offset = 0.25
        
        self.rotor_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=rotor_size[0], length=rotor_size[2], 
                                              rgbaColor=rotor_color)
        
        # Front left rotor
        self.rotor_fl = p.createMultiBody(baseMass=0, baseVisualShapeIndex=self.rotor_visual,
                                         basePosition=[self.start_pos[0] - rotor_offset, 
                                                      self.start_pos[1] + rotor_offset, 
                                                      self.start_pos[2] + 0.1])
        
        # Front right rotor
        self.rotor_fr = p.createMultiBody(baseMass=0, baseVisualShapeIndex=self.rotor_visual,
                                         basePosition=[self.start_pos[0] + rotor_offset, 
                                                      self.start_pos[1] + rotor_offset, 
                                                      self.start_pos[2] + 0.1])
        
        # Rear left rotor
        self.rotor_rl = p.createMultiBody(baseMass=0, baseVisualShapeIndex=self.rotor_visual,
                                         basePosition=[self.start_pos[0] - rotor_offset, 
                                                      self.start_pos[1] - rotor_offset, 
                                                      self.start_pos[2] + 0.1])
        
        # Rear right rotor
        self.rotor_rr = p.createMultiBody(baseMass=0, baseVisualShapeIndex=self.rotor_visual,
                                         basePosition=[self.start_pos[0] + rotor_offset, 
                                                      self.start_pos[1] - rotor_offset, 
                                                      self.start_pos[2] + 0.1])
        
        # Create fixed constraints to attach rotors to drone
        p.createConstraint(self.drone_id, -1, self.rotor_fl, -1, p.JOINT_FIXED, [0,0,0], [rotor_offset, -rotor_offset, -0.1], [0,0,0])
        p.createConstraint(self.drone_id, -1, self.rotor_fr, -1, p.JOINT_FIXED, [0,0,0], [-rotor_offset, -rotor_offset, -0.1], [0,0,0])
        p.createConstraint(self.drone_id, -1, self.rotor_rl, -1, p.JOINT_FIXED, [0,0,0], [rotor_offset, rotor_offset, -0.1], [0,0,0])
        p.createConstraint(self.drone_id, -1, self.rotor_rr, -1, p.JOINT_FIXED, [0,0,0], [-rotor_offset, rotor_offset, -0.1], [0,0,0])
    
    def update(self):
        # Update battery
        self.battery = max(0, self.battery - self.battery_drain_rate * (1 + 0.5 * self.carrying_package))
        
        # Get current position and velocity
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        self.current_pos = list(pos)
        vel, _ = p.getBaseVelocity(self.drone_id)
        
        if self.target_pos and not self.reached_target:
            # Calculate control forces using PID
            force_x = self.pid_x.update(self.target_pos[0], self.current_pos[0], vel[0])
            force_y = self.pid_y.update(self.target_pos[1], self.current_pos[1], vel[1])
            force_z = self.pid_z.update(self.target_pos[2], self.current_pos[2], vel[2])
            
            # Apply forces
            p.applyExternalForce(self.drone_id, -1, [force_x, force_y, force_z], [0,0,0], p.WORLD_FRAME)
            
            # Check if reached target
            distance = np.linalg.norm(np.array(self.current_pos) - np.array(self.target_pos))
            if distance < 0.5:  # Threshold distance
                if self.path:
                    self.target_pos = self.path.popleft()
                else:
                    self.reached_target = True
    
    def set_target(self, target_pos):
        self.target_pos = target_pos.copy()
        self.target_pos[2] = max(2.0, self.target_pos[2])  # Minimum flight height
        self.reached_target = False
        
        # Simple pathfinding - just go directly to target in this basic version
        self.path = deque()
    
    def pick_up_package(self, package_id):
        if not self.carrying_package:
            # Create constraint to attach package to drone
            self.current_package_id = package_id
            p.createConstraint(self.drone_id, -1, package_id, -1, p.JOINT_FIXED, [0,0,0], [0,0,-0.2], [0,0,0])
            self.carrying_package = True
            self.battery_drain_rate *= 1.5  # Increase battery drain when carrying package
    
    def drop_package(self):
        if self.carrying_package:
            # Remove constraint
            for constraint in p.getConstraintUniqueIds():
                info = p.getConstraintInfo(constraint)
                if info[2] == self.drone_id and info[3] == self.current_package_id:
                    p.removeConstraint(constraint)
                    break
            
            self.carrying_package = False
            self.current_package_id = None
            self.battery_drain_rate /= 1.5  # Reset battery drain rate
    
    def get_state(self):
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        vel, ang_vel = p.getBaseVelocity(self.drone_id)
        
        return {
            'position': pos,
            'orientation': orn,
            'velocity': vel,
            'angular_velocity': ang_vel,
            'battery': self.battery,
            'carrying_package': self.carrying_package,
            'target_position': self.target_pos,
            'reached_target': self.reached_target
        }

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
        self.prev_error = 0
        self.integral = 0
    
    def update(self, target, current, derivative_input=None):
        error = target - current
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error
        i_term = self.ki * self.integral
        
        # Derivative term
        if derivative_input is not None:
            d_term = self.kd * (-derivative_input)  # Negative since derivative_input is velocity
        else:
            d_term = self.kd * (error - self.prev_error)
            self.prev_error = error
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(min(output, self.max_output), -self.max_output)
        
        return output