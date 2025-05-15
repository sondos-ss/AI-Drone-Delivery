from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from direct.task import Task
from direct.gui.OnscreenText import OnscreenText
import random
import heapq
import math
import sys
from algos import a_star, bfs, dfs, ucs, gbfs, bidirectional

drones = sys.argv[1] if len(sys.argv) > 1 else 1
algo = sys.argv[2] if len(sys.argv) > 2 else 'a_star'
heur = sys.argv[3] if len(sys.argv) > 3 else 'manhatten'

# print(drones, algo, heur)

# Constants
GRID_SIZE = 30
CELL_SIZE = 1
NUM_PACKAGES = 12
FLY_HEIGHT = 3.0
GROUND_HEIGHT = 0.5
DRONE_SPEED = 8.0
BATTERY_CAPACITY = 100.0
NUM_DRONES = int(drones)  # Number of drones to simulate

class DroneDeliverySim(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.disableMouse()

        # Game state
        self.grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        self.buildings = set()
        self.packages = []
        self.drones = []  # List to store all drones
        self.score = 0
        self.delivered_count = 0
        self.building_obstacles = set()
        self.current_path = []
        self.current_package = None
        self.path_index = 0
        self.battery = BATTERY_CAPACITY
        self.rotor_angle = 0

        # Setup
        self.setup_world()
        self.setup_ui()
        self.taskMgr.add(self.update, "update")

    def setup_world(self):
        """Initialize the 3D world environment"""
        # Ground with grid pattern
        for x in range(GRID_SIZE):
            for y in range(GRID_SIZE):
                tile = self.loader.loadModel("models/box")
                tile.setScale(0.95, 0.95, 0.05)
                tile.setPos(x, y, -0.05)
                # Alternate colors for grid pattern
                if (x + y) % 2 == 0:
                    tile.setColor(0.95, 0.95, 1.0, 1)  # Light blue
                else:
                    tile.setColor(0.85, 0.85, 0.95, 1)  # Slightly darker
                tile.reparentTo(self.render)

        # Buildings with varying heights
        building_colors = [
            (0.6, 0.6, 0.7, 1),  # Gray-blue
            (0.7, 0.7, 0.8, 1),   # Lighter gray-blue
            (0.5, 0.6, 0.6, 1)    # Blue-gray
        ]
        
        # Generate buildings
        for x in range(0, GRID_SIZE, 3):
            for y in range(0, GRID_SIZE, 3):
                if random.random() < 0.5:  # 50% chance of building
                    height = random.uniform(1.5, 4.0)
                    building = self.loader.loadModel("models/box")
                    building.setScale(0.9, 0.9, height)
                    building.setPos(x + 0.5, y + 0.5, height/2)
                    building.setColor(random.choice(building_colors))
                    building.reparentTo(self.render)
                    self.buildings.add((x, y))
                    for dx in range(-1, 1):
                        for dy in range(-1, 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                                self.building_obstacles.add((nx, ny))
                    
                    # Add windows
                    if random.random() > 0.2:
                        for _ in range(int(height)):
                            window = self.loader.loadModel("models/box")
                            window.setScale(0.2, 0.05, 0.2)
                            window.setColor(0.9, 0.9, 0.7, 1)
                            window.setPos(
                                random.uniform(-0.3, 0.3),
                                random.uniform(-0.3, 0.3),
                                random.uniform(-height/2 + 0.5, height/2 - 0.5)
                            )
                            window.reparentTo(building)

        # Generate all valid non-building positions
        valid_positions = []
        for x in range(GRID_SIZE):
            for y in range(GRID_SIZE):
                if (x, y) not in self.buildings:
                    valid_positions.append((x, y))
        
        # Ensure we have enough positions for all packages
        if len(valid_positions) < NUM_PACKAGES * 2:
            print("Error: Not enough valid positions for packages!")
            return

        # Shuffle and create package pairs
        random.shuffle(valid_positions)
        
        # Create packages with connected source-destination pairs
        for i in range(NUM_PACKAGES):
            start_pos = valid_positions[i*2]
            end_pos = valid_positions[i*2 + 1]
            
            # Create pickup point
            pickup = self.loader.loadModel("models/box")
            pickup.setScale(0.4, 0.4, 0.2)
            pickup.setColor(random.choice([
                (0.1, 0.4, 1, 1),   # Blue
                (1, 0.2, 0.2, 1),   # Red
                (0.8, 0.8, 0, 1)    # Yellow
            ]))
            pickup.setPos(start_pos[0] + 0.5, start_pos[1] + 0.5, 0.2)
            pickup.reparentTo(self.render)

            # Create delivery point
            goal = self.loader.loadModel("models/box")
            goal.setScale(0.6, 0.6, 0.1)
            goal.setColor(0.2, 1, 0.2, 1)
            goal.setPos(end_pos[0] + 0.5, end_pos[1] + 0.5, 0.1)
            goal.reparentTo(self.render)

            # Add marker pole
            pole = self.loader.loadModel("models/box")
            pole.setScale(0.1, 0.1, 1.0)
            pole.setColor(0.8, 0.8, 0.8, 1)
            pole.setPos(end_pos[0] + 0.5, end_pos[1] + 0.5, 0.5)
            pole.reparentTo(self.render)

            self.packages.append({
                'start': start_pos,
                'goal': end_pos,
                'pickup_model': pickup,
                'goal_model': goal,
                'pole_model': pole,
                'picked': False,
                'delivered': False,
                'assigned_to': None  # Track which drone is assigned to this package
            })

        # Create multiple drones
        drone_colors = [
            (1, 0.3, 0.3, 1),  # Red
            (0.3, 1, 0.3, 1),  # Green
            (0.3, 0.3, 1, 1)   # Blue
        ]
        
        for i in range(NUM_DRONES):
            drone = {
                'model': self.loader.loadModel("models/box"),
                'rotors': [],
                'current_path': [],
                'current_package': None,
                'path_index': 0,
                'battery': BATTERY_CAPACITY,
                'rotor_angle': 0,
                'id': i
            }
            
            drone['model'].setScale(0.5, 0.5, 0.2)
            drone['model'].setColor(drone_colors[i % len(drone_colors)])
            # Start drones at different positions
            drone['model'].setPos(i * 2, 0, FLY_HEIGHT)
            drone['model'].reparentTo(self.render)
            
            # Add rotors
            for x, y in [(0.3, 0.3), (0.3, -0.3), (-0.3, 0.3), (-0.3, -0.3)]:
                rotor = self.loader.loadModel("models/box")
                rotor.setScale(0.15, 0.15, 0.05)
                rotor.setColor(0.2, 0.2, 0.2, 1)
                rotor.setPos(x, y, 0.15)
                rotor.reparentTo(drone['model'])
                drone['rotors'].append(rotor)
            
            self.drones.append(drone)
            self.taskMgr.add(lambda task, i=i: self.update_battery(task, i), f"update_battery_{i}")

        # Camera setup (now follows first drone)
        self.camera.setPos(GRID_SIZE / 2, -GRID_SIZE * 1.2, GRID_SIZE * 0.8)
        self.camera.lookAt(GRID_SIZE / 2, GRID_SIZE / 2, 0)

        # Lighting
        ambient = AmbientLight("ambient")
        ambient.setColor((0.4, 0.4, 0.4, 1))
        self.render.setLight(self.render.attachNewNode(ambient))

        directional = DirectionalLight("dir")
        directional.setColor((0.9, 0.9, 0.8, 1))
        directional_np = self.render.attachNewNode(directional)
        directional_np.setHpr(45, -60, 0)
        self.render.setLight(directional_np)

    def setup_ui(self):
        """Initialize the user interface elements"""
        # Battery displays (one for each drone)
        self.battery_texts = []
        for i in range(NUM_DRONES):
            text = OnscreenText(
                text=f"Drone {i+1}: {BATTERY_CAPACITY}%",
                pos=(-1.3, 0.9 - i * 0.08),
                scale=0.07,
                fg=(1, 1, 1, 1),
                align=TextNode.ALeft,
                mayChange=True
            )
            self.battery_texts.append(text)
        
        # Score display
        self.score_text = OnscreenText(
            text=f"Score: {self.score}",
            pos=(1.0, 0.9),
            scale=0.07,
            fg=(1, 1, 1, 1),
            align=TextNode.ARight,
            mayChange=True
        )
        
        # Packages delivered
        self.delivered_text = OnscreenText(
            text=f"Delivered: {self.delivered_count}/{NUM_PACKAGES}",
            pos=(0, 0.9),
            scale=0.07,
            fg=(1, 1, 1, 1),
            align=TextNode.ACenter,
            mayChange=True
        )

    def update(self, task):
        """Main game update loop"""
        dt = globalClock.getDt()

        # Update each drone
        for drone in self.drones:
            # Rotate rotors
            self.rotate_rotors(drone, dt)

            # Find new target if needed
            if not drone['current_path'] and drone['current_package'] is None:
                self.find_next_package(drone)

            # Follow current path
            if drone['current_path'] and drone['path_index'] < len(drone['current_path']):
                self.follow_path(drone, dt)
            elif drone['current_package']:
                self.handle_package(drone)

        # Update camera to follow first drone
       # self.update_camera()

        return Task.cont

    def rotate_rotors(self, drone, dt):
        """Animate the drone rotors"""
        drone['rotor_angle'] = (drone['rotor_angle'] + 500 * dt) % 360
        for rotor in drone['rotors']:
            rotor.setH(drone['rotor_angle'])

    def find_next_package(self, drone):
        """Find the next package to pick up or deliver for this drone"""
        # First check if this drone has a package that needs delivering
        for pkg in self.packages:
            if pkg['picked'] and not pkg['delivered'] and pkg['assigned_to'] == drone['id']:
                # Find path to delivery point
                drone['current_package'] = pkg
                drone['current_path'] = self.path_gen(algo, self.get_grid_pos(), pkg['start'], heur)
                drone['path_index'] = 0
                if not drone['current_path']:
                    print(f"Drone {drone['id']}: No path to delivery point found!")
                    drone['current_package'] = None
                    drone['current_path'] = []
                return

        # Find the nearest unassigned package
        unassigned_pkgs = [pkg for pkg in self.packages if not pkg['picked'] and pkg['assigned_to'] is None]
        if not unassigned_pkgs:
            return

        # Calculate distances from drone to each package
        drone_pos = self.get_grid_pos(drone)
        distances = []
        for pkg in unassigned_pkgs:
            dist = abs(drone_pos[0] - pkg['start'][0]) + abs(drone_pos[1] - pkg['start'][1])
            distances.append((dist, pkg))

        # Sort by distance and find the closest package that isn't assigned to another drone
        distances.sort(key=lambda x: x[0])
        for dist, pkg in distances:
            if pkg['assigned_to'] is None:
                # Assign this package to current drone
                pkg['assigned_to'] = drone['id']
                drone['current_package'] = pkg
                drone['current_path'] = self.path_gen(algo, drone_pos, pkg['start'], heur)
                # drone['current_path'] = self.a_star(drone_pos, pkg['start'])                
                drone['path_index'] = 0
                if not drone['current_path']:
                    print(f"Drone {drone['id']}: No path to package found!")
                    drone['current_package'] = None
                    drone['current_path'] = []
                    pkg['assigned_to'] = None
                break

    def follow_path(self, drone, dt):
        """Move the drone along the current path"""
        tx, ty = drone['current_path'][drone['path_index']]
        is_target = (drone['path_index'] == len(drone['current_path']) - 1)
        target_z = GROUND_HEIGHT if is_target else FLY_HEIGHT
        target_pos = LPoint3f(tx + 0.5, ty + 0.5, target_z)
        
        current_pos = drone['model'].getPos()
        direction = target_pos - current_pos
        distance = direction.length()
        
        if distance > 0.1:
            direction.normalize()
            move_dist = min(DRONE_SPEED * dt, distance)
            drone['model'].setPos(current_pos + direction * move_dist)
            
            # Smooth rotation toward movement direction
            target_hpr = Vec3(math.degrees(math.atan2(-direction.getX(), direction.getY())), 0, 0)
            current_hpr = drone['model'].getHpr()
            new_hpr = current_hpr + (target_hpr - current_hpr) * 5 * dt
            drone['model'].setHpr(new_hpr)
        else:
            drone['path_index'] += 1

    def handle_package(self, drone):
        """Handle package pickup or delivery for this drone"""
        drone_pos = self.get_grid_pos(drone)
        pkg = drone['current_package']

        if not pkg['picked'] and drone_pos == pkg['start']:
            # Pick up package
            pkg['picked'] = True
            pkg['pickup_model'].removeNode()
            print(f"Drone {drone['id']}: 📦 Package picked up!")
            self.score += 10
            self.score_text.setText(f"Score: {self.score}")
            
            # Find path to delivery point
            drone['current_path'] = self.path_gen(algo, drone_pos, pkg['goal'], heur)
            drone['path_index'] = 0
            if not drone['current_path']:
                print(f"Drone {drone['id']}: No path to delivery point!")
                drone['current_package'] = None
                drone['current_path'] = []

        elif pkg['picked'] and not pkg['delivered'] and drone_pos == pkg['goal']:
            # Deliver package
            pkg['delivered'] = True
            pkg['goal_model'].removeNode()
            pkg['pole_model'].removeNode()
            print(f"Drone {drone['id']}: ✅ Package delivered!")
            self.score += 50
            self.delivered_count += 1
            drone['battery'] = min(BATTERY_CAPACITY, drone['battery'] + 10)
            
            # Update UI
            self.score_text.setText(f"Score: {self.score}")
            self.delivered_text.setText(f"Delivered: {self.delivered_count}/{NUM_PACKAGES}")
            
            # Clear current package and path
            drone['current_package'] = None
            drone['current_path'] = []
            pkg['assigned_to'] = None
            
            # Check if all packages delivered
            if self.delivered_count == NUM_PACKAGES:
                print("🎉 All packages delivered! Mission complete!")
            else:
                # Immediately look for next package
                self.find_next_package(drone)

    def update_battery(self, task, drone_id):
        """Update battery level for a specific drone"""
        drone = self.drones[drone_id]
    
        # Only decrease battery if there are still packages to deliver
        if self.delivered_count < NUM_PACKAGES:
            drone['battery'] -= 0.03  # Battery drain rate
            if drone['battery'] <= 0:
                print(f"⚠️ Drone {drone_id} battery depleted!")
                drone['battery'] = 0
                # Remove this drone from active duty
                if drone['current_package']:
                    drone['current_package']['assigned_to'] = None
                drone['current_package'] = None
                drone['current_path'] = []
    
        # Update battery display
        self.battery_texts[drone_id].setText(f"Drone {drone_id+1}: {int(drone['battery'])}%")
    
        # Change color when battery is low
        if drone['battery'] < 20:
            self.battery_texts[drone_id].setColor(1, 0, 0, 1)
        else:
            self.battery_texts[drone_id].setColor(1, 1, 1, 1)
    
        return Task.cont

    def get_grid_pos(self, drone):
        """Get drone's grid position (x,y)"""
        pos = drone['model'].getPos()
        return (int(pos.getX()), int(pos.getY()))
      
    def path_gen(self, algo, start, goal, heur):
        if algo == 'a_star':
            return a_star(start, goal, heur, GRID_SIZE, self.buildings)
        elif algo == 'bfs':
            return bfs(start, goal, GRID_SIZE, self.buildings)
        elif algo == 'dfs':
            return dfs(start, goal, GRID_SIZE, self.buildings)
        elif algo == 'ucs':
            return ucs(start, goal, GRID_SIZE, self.buildings)
        elif algo == 'gbfs':
            return gbfs(start, goal, heur, GRID_SIZE, self.buildings)
        elif algo == 'bidirectional':
            return bidirectional(start, goal, GRID_SIZE, self.buildings)
        return algo(start, goal, heur)

app = DroneDeliverySim()
app.run()