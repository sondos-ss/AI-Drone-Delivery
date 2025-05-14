import numpy as np
from collections import deque

class DeliveryManager:
    def __init__(self, env):
        self.env = env
        self.tasks = []
        self.completed_tasks = []
        self.current_task = None
        self.task_queue = deque()
        
        # Delivery statistics
        self.total_deliveries = 0
        self.total_distance = 0.0
        self.total_time = 0.0
    
    def add_task(self, pickup_pos, delivery_pos):
        task_id = self.env.add_delivery_task(pickup_pos, delivery_pos)
        self.tasks.append({
            'id': task_id,
            'pickup_pos': pickup_pos,
            'delivery_pos': delivery_pos,
            'status': 'pending',
            'distance': np.linalg.norm(np.array(delivery_pos) - np.array(pickup_pos))
        })
        self.task_queue.append(task_id)
        return task_id
    
    def assign_next_task(self, drone):
        if self.current_task is None and self.task_queue:
            task_id = self.task_queue.popleft()
            self.current_task = self.tasks[task_id]
            self.current_task['status'] = 'assigned'
            
            # Set drone target to pickup location
            pickup_pos = self.current_task['pickup_pos'].copy()
            pickup_pos[2] = 2.0  # Fly at height 2 for pickup
            drone.set_target(pickup_pos)
            
            return True
        return False
    
    def update_task_state(self, drone):
        if self.current_task is None:
            return False
        
        task = self.current_task
        drone_state = drone.get_state()
        
        # Check if drone reached pickup location
        if task['status'] == 'assigned' and drone_state['reached_target']:
            # Pick up package
            package_id = self.env.packages[task['id']][0]
            drone.pick_up_package(package_id)
            task['status'] = 'picked_up'
            
            # Set drone target to delivery location
            delivery_pos = task['delivery_pos'].copy()
            delivery_pos[2] = 2.0  # Fly at height 2 for delivery
            drone.set_target(delivery_pos)
            return True
        
        # Check if drone reached delivery location
        elif task['status'] == 'picked_up' and drone_state['reached_target']:
            # Drop package
            drone.drop_package()
            task['status'] = 'delivered'
            task['completion_time'] = self.total_time
            
            # Update statistics
            self.total_deliveries += 1
            self.total_distance += task['distance']
            
            # Move to completed tasks
            self.completed_tasks.append(task)
            self.current_task = None
            return True
        
        return False
    
    def update(self, drone, dt):
        self.total_time += dt
        
        # Update current task state
        task_updated = self.update_task_state(drone)
        
        # Assign new task if drone is free
        if self.current_task is None:
            self.assign_next_task(drone)
        
        return task_updated
    
    def get_stats(self):
        avg_time = np.mean([t['completion_time'] for t in self.completed_tasks]) if self.completed_tasks else 0
        avg_distance = np.mean([t['distance'] for t in self.completed_tasks]) if self.completed_tasks else 0
        
        return {
            'total_deliveries': self.total_deliveries,
            'total_distance': self.total_distance,
            'total_time': self.total_time,
            'avg_delivery_time': avg_time,
            'avg_delivery_distance': avg_distance,
            'pending_tasks': len(self.task_queue),
            'completed_tasks': len(self.completed_tasks)
        }