from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from direct.task import Task
from direct.gui.OnscreenText import OnscreenText
import random
import heapq
import math

# Constants
GRID_SIZE = 30
CELL_SIZE = 1
NUM_PACKAGES = 5
FLY_HEIGHT = 3.0
GROUND_HEIGHT = 0.5
DRONE_SPEED = 8.0
BATTERY_CAPACITY = 100.0

class DroneDeliverySim(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.disableMouse()

        # Game state
        self.grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        self.buildings = set()
        self.packages = []
        self.current_path = []
        self.current_package = None
        self.path_index = 0
        self.battery = BATTERY_CAPACITY
        self.score = 0
        self.delivered_count = 0
        self.rotor_angle = 0

        # Setup
        self.setup_world()
        self.setup_ui()
        self.taskMgr.add(self.update, "update")
        self.taskMgr.add(self.update_battery, "update_battery")

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
                'delivered': False
            })

        # Drone model with rotors
        self.drone = self.loader.loadModel("models/box")
        self.drone.setScale(0.5, 0.5, 0.2)
        self.drone.setColor(1, 0.3, 0.3, 1)
        self.drone.setPos(0, 0, FLY_HEIGHT)
        self.drone.reparentTo(self.render)
        
        # Add rotors (4 small boxes)
        self.rotors = []
        for i, (x, y) in enumerate([(0.3, 0.3), (0.3, -0.3), (-0.3, 0.3), (-0.3, -0.3)]):
            rotor = self.loader.loadModel("models/box")
            rotor.setScale(0.15, 0.15, 0.05)
            rotor.setColor(0.2, 0.2, 0.2, 1)
            rotor.setPos(x, y, 0.15)
            rotor.reparentTo(self.drone)
            self.rotors.append(rotor)

        # Camera setup
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
        # Battery display
        self.battery_text = OnscreenText(
            text=f"Battery: {int(self.battery)}%",
            pos=(-1.3, 0.9),
            scale=0.07,
            fg=(1, 1, 1, 1),
            align=TextNode.ALeft,
            mayChange=True
        )
        
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

        # Rotate rotors
        self.rotate_rotors(dt)

        # Find new target if needed
        if not self.current_path and self.current_package is None:
            self.find_next_package()

        # Follow current path
        if self.current_path and self.path_index < len(self.current_path):
            self.follow_path(dt)
        elif self.current_package:
            self.handle_package()

        # Update camera to follow drone
        self.update_camera()

        return Task.cont

    def rotate_rotors(self, dt):
        """Animate the drone rotors"""
        self.rotor_angle = (self.rotor_angle + 500 * dt) % 360
        for rotor in self.rotors:
            rotor.setH(self.rotor_angle)

    def find_next_package(self):
        """Find the next package to pick up or deliver"""
        for pkg in self.packages:
            if not pkg['picked']:
                # Find path to package
                self.current_package = pkg
                self.current_path = self.a_star(self.get_grid_pos(), pkg['start'])
                self.path_index = 0
                if not self.current_path:
                    print("No path to package found!")
                    self.current_package = None
                    self.current_path = []
                break
            elif pkg['picked'] and not pkg['delivered']:
                # Find path to delivery point
                self.current_package = pkg
                self.current_path = self.a_star(self.get_grid_pos(), pkg['goal'])
                self.path_index = 0
                if not self.current_path:
                    print("No path to delivery point found!")
                    self.current_package = None
                    self.current_path = []
                break

    def follow_path(self, dt):
        """Move the drone along the current path"""
        tx, ty = self.current_path[self.path_index]
        is_target = (self.path_index == len(self.current_path) - 1)
        target_z = GROUND_HEIGHT if is_target else FLY_HEIGHT
        target_pos = LPoint3f(tx + 0.5, ty + 0.5, target_z)
        
        current_pos = self.drone.getPos()
        direction = target_pos - current_pos
        distance = direction.length()
        
        if distance > 0.1:
            direction.normalize()
            move_dist = min(DRONE_SPEED * dt, distance)
            self.drone.setPos(current_pos + direction * move_dist)
            
            # Smooth rotation toward movement direction
            target_hpr = Vec3(math.degrees(math.atan2(-direction.getX(), direction.getY())), 0, 0)
            current_hpr = self.drone.getHpr()
            new_hpr = current_hpr + (target_hpr - current_hpr) * 5 * dt
            self.drone.setHpr(new_hpr)
        else:
            self.path_index += 1

    def handle_package(self):
        """Handle package pickup or delivery"""
        drone_pos = self.get_grid_pos()
        pkg = self.current_package

        if not pkg['picked'] and drone_pos == pkg['start']:
            # Pick up package
            pkg['picked'] = True
            pkg['pickup_model'].removeNode()
            print("📦 Package picked up!")
            self.score += 10
            self.score_text.setText(f"Score: {self.score}")
            
            # Find path to delivery point
            self.current_path = self.a_star(drone_pos, pkg['goal'])
            self.path_index = 0
            if not self.current_path:
                print("No path to delivery point!")
                self.current_package = None
                self.current_path = []

        elif pkg['picked'] and not pkg['delivered'] and drone_pos == pkg['goal']:
            # Deliver package
            pkg['delivered'] = True
            pkg['goal_model'].removeNode()
            pkg['pole_model'].removeNode()
            print("✅ Package delivered!")
            self.score += 50
            self.delivered_count += 1
            self.battery = min(BATTERY_CAPACITY, self.battery + 10)
            
            # Update UI
            self.score_text.setText(f"Score: {self.score}")
            self.delivered_text.setText(f"Delivered: {self.delivered_count}/{NUM_PACKAGES}")
            
            # Clear current package and path
            self.current_package = None
            self.current_path = []
            
            # Check if all packages delivered
            if self.delivered_count == NUM_PACKAGES:
                print("🎉 All packages delivered! Mission complete!")
            else:
                # Immediately look for next package
                self.find_next_package()

    def update_battery(self, task):
        """Update battery level and check for depletion"""
        self.battery -= 0.03  # Battery drain rate
        if self.battery <= 0:
            print("⚠️ Battery depleted! Mission failed.")
            self.battery = 0
            self.taskMgr.remove("update")  # Stop the simulation
        
        # Update battery display
        self.battery_text.setText(f"Battery: {int(self.battery)}%")
        
        # Change color when battery is low
        if self.battery < 20:
            self.battery_text.setColor(1, 0, 0, 1)  # Red
        else:
            self.battery_text.setColor(1, 1, 1, 1)  # White
        
        return Task.cont

    def update_camera(self):
        """Update camera position to follow drone"""
        drone_pos = self.drone.getPos()
        target_pos = LPoint3f(
            drone_pos.getX(),
            drone_pos.getY() - 15,
            drone_pos.getZ() + 10
        )
        current_pos = self.camera.getPos()
        self.camera.setPos(current_pos + (target_pos - current_pos) * 0.05)
        self.camera.lookAt(drone_pos)

    def get_grid_pos(self):
        """Get drone's grid position (x,y)"""
        pos = self.drone.getPos()
        return (int(pos.getX()), int(pos.getY()))

    def a_star(self, start, goal):
        """A* pathfinding algorithm"""
        def heuristic(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])  # Manhattan distance

        open_set = []
        heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start, []))
        visited = set()

        while open_set:
            est_total, cost, current, path = heapq.heappop(open_set)
            
            if current in visited:
                continue
                
            visited.add(current)
            
            if current == goal:
                return path + [current]

            # Explore neighbors (including diagonals)
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1), (-1,-1),(-1,1),(1,-1),(1,1)]:
                nx, ny = current[0]+dx, current[1]+dy
                
                # Check boundaries and obstacles
                if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and 
                    (nx, ny) not in self.buildings):
                    
                    # Diagonal movement costs more
                    move_cost = 1.4 if dx != 0 and dy != 0 else 1.0
                    heapq.heappush(
                        open_set, 
                        (cost + move_cost + heuristic((nx, ny), goal), 
                         cost + move_cost, 
                         (nx, ny), 
                         path + [current])
                    )

        return []  # No path found

app = DroneDeliverySim()
app.run()