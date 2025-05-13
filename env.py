import pybullet as p
import pybullet_data
import numpy as np

class DeliveryEnvironment:
    def __init__(self, gui=True):
        self.gui = gui
        self.physicsClient = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Environment parameters
        self.ground_size = [50, 50]
        self.buildings = []
        self.packages = []
        self.destinations = []
        
        self.setup_world()
    
    def setup_world(self):
        # Load ground plane
        self.ground_id = p.loadURDF("plane.urdf")
        
        # Create simple city layout
        self.create_buildings()
        
        # Visualize world boundaries
        self.create_boundaries()
    
    def create_buildings(self):
        # Create some simple buildings as obstacles
        for i in range(10):
            building_pos = [np.random.uniform(-20, 20), 
                          np.random.uniform(-20, 20), 
                          0]
            building_size = [np.random.uniform(2, 5), 
                            np.random.uniform(2, 5), 
                            np.random.uniform(5, 15)]
            
            building_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=building_size)
            building_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=building_size, rgbaColor=[0.5, 0.5, 0.5, 1])
            building_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=building_shape, 
                                          baseVisualShapeIndex=building_visual, basePosition=building_pos)
            self.buildings.append((building_id, building_pos, building_size))
    
    def create_boundaries(self):
        # Create visual boundaries for the world
        boundary_height = 0.1
        boundary_thickness = 0.5
        
        # Create boundary walls
        boundary_size_x = [self.ground_size[0], boundary_thickness, boundary_height]
        boundary_size_y = [boundary_thickness, self.ground_size[1], boundary_height]
        
        # North wall
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=boundary_size_x),
                         baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=boundary_size_x, rgbaColor=[1,0,0,0.5]),
                         basePosition=[0, self.ground_size[1]/2, 0])
        
        # South wall
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=boundary_size_x),
                         baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=boundary_size_x, rgbaColor=[1,0,0,0.5]),
                         basePosition=[0, -self.ground_size[1]/2, 0])
        
        # East wall
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=boundary_size_y),
                         baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=boundary_size_y, rgbaColor=[1,0,0,0.5]),
                         basePosition=[self.ground_size[0]/2, 0, 0])
        
        # West wall
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=boundary_size_y),
                         baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=boundary_size_y, rgbaColor=[1,0,0,0.5]),
                         basePosition=[-self.ground_size[0]/2, 0, 0])
    
    def add_delivery_task(self, pickup_pos, delivery_pos):
        # Create package at pickup location
        package_size = [0.2, 0.2, 0.1]
        package_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=package_size)
        package_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=package_size, rgbaColor=[0,1,0,1])
        package_id = p.createMultiBody(baseMass=0.5, baseCollisionShapeIndex=package_shape,
                                     baseVisualShapeIndex=package_visual, basePosition=pickup_pos)
        
        # Create visual marker at delivery location
        marker_size = [0.3, 0.3, 0.01]
        marker_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=marker_size, rgbaColor=[1,0,0,0.7])
        marker_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=marker_visual, basePosition=delivery_pos)
        
        self.packages.append((package_id, pickup_pos))
        self.destinations.append((marker_id, delivery_pos))
        
        return len(self.packages) - 1  # Return task ID
    
    def step_simulation(self):
        p.stepSimulation()
    
    def close(self):
        p.disconnect()