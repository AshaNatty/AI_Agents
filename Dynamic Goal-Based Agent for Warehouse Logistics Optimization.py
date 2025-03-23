import random
import heapq

random.seed(54)

def generate_random_warehouse():
    """
    Generates a random warehouse grid and associated parameters,
    labeling:
      - Empty cells as " "
      - Obstacles as "X"
      - Packages as "P1", "P2", ...
      - Drop-off locations as "D1", "D2", ...
    Robot start is always (0,0), which remains " ".

    Returns a dictionary containing:
      - rows, cols
      - grid (2D list of strings)
      - num_packages, num_obstacles
      - package_locations, drop_off_locations
      - robot_start (row, col)

    Conditions:
      1. 5 <= N, M <= 10
      2. 1 <= O <= 10   (number of obstacles)
      3. 2 <= P <= 6    (number of packages)
      4. Packages and drop-offs do not overlap each other or obstacles
      5. Robot start is always (0,0) and remains free
    """

    # 1) Random warehouse dimensions: N x M
    N = random.randint(5, 10)
    M = random.randint(5, 10)

    # Initialize the grid to " " (empty space)
    grid = [[" " for _ in range(M)] for _ in range(N)]

    # Keep track of used positions to avoid overlap
    # Mark (0,0) as used for the robot start so we don't place anything else there
    used_positions = {(0, 0)}


    # 3) Random number of packages
    P = random.randint(2, 6)
    package_locations = []
    drop_off_locations = []


    # Place packages and drop-offs without overlap
    for i in range(P):
        # Find a free cell for the package
        while True:
            r = random.randint(0, N - 1)
            c = random.randint(0, M - 1)
            if (r, c) not in used_positions:
                label = f"P{i+1}"
                grid[r][c] = label
                used_positions.add((r, c))
                package_locations.append((r, c))
                break

        # Find a free cell for the drop-off
        while True:
            r = random.randint(0, N - 1)
            c = random.randint(0, M - 1)
            if (r, c) not in used_positions:
                label = f"D{i+1}"
                grid[r][c] = label
                used_positions.add((r, c))
                drop_off_locations.append((r, c))
                break

    # 2) Random number of obstacles
    O = random.randint(1, 10)

    # Place static obstacles (marked as "X")
    count_obstacles = 0
    obstacle_locations = []
    while count_obstacles < O:
        r = random.randint(0, N - 1)
        c = random.randint(0, M - 1)
        if (r, c) not in used_positions:
            grid[r][c] = "X"
            used_positions.add((r, c))
            obstacle_locations.append((r,c))
            count_obstacles += 1

    # 4) Robot start is always (0,0) => keep as " "
    robot_start = (0, 0)

    return {
        "rows": N,
        "cols": M,
        "grid": grid,
        "num_packages": P,
        "num_obstacles": O,
        "package_locations": package_locations,
        "drop_off_locations": drop_off_locations,
        "obstacle_locations": obstacle_locations,
        "robot_start": robot_start
    }
def show_grid(grid):
    for idx, row in enumerate(grid):
        print(f"{row}")
class GoalBasedAgent:
    def __init__(self, environment):
        self.environment = environment
        self.state = "FIND_PICKUP_POINT"
        self.location = environment["robot_start"]
        self.current_goal =(0,0)
        self.path = []
        self.cost = 0
        self.reward = 0
    
    def perceive(self):
        if self.location in self.environment["obstacle_locations"]:
            return "OBSTACLE"
        elif self.location in self.environment["drop_off_locations"]:
            return "DROP POINT"
        elif self.location in self.environment["package_locations"]:
            return "PICKUP POINT"
        else:
            return "EMPTY"
            
        
    def move(self, new_location):
        self.location = new_location
        self.cost += 1

    def act(self):
        
        percept  = self.perceive()
        if percept == "PICKUP POINT" and self.current_goal == self.location:
            print(f"Package picked from  {self.location}.")
            self.environment["package_locations"].remove(self.location)
            self.state = "FIND_DROP_POINT"
        elif percept == "DROP POINT" and self.current_goal == self.location:
            print(f"Package dropped to {self.location}.")
            self.environment["drop_off_locations"].remove(self.location)
            self.reward = self.reward + 10
            # If we still have packages left, go find another pickup; otherwise, we are done
            if len(self.environment["package_locations"]) > 0:
                self.state = "FIND_PICKUP_POINT"
            else:
                self.state = "COMPLETED"
        elif percept == "OBSTACLE":
            print(f"Hit a obstacle at  {self.location}. Penalty incurred -5")
            self.cost = self.cost + 5
            self.move(self.path[0])
            self.path = self.path[1:]
        else:
            # Skip any leading positions in the path that match the current location
            while len(self.path) > 0 and self.path[0] == self.location:
                self.path.pop(0)  # Remove the first element since it's the same as the current location
            
            # Now, if there's still something in the path, move there
            if len(self.path) > 0:
                print(f"Moving from {self.location} to {self.path[0]}")
                self.move(self.path[0])
                self.path.pop(0)

        self.compute_goal()

    def find_shortest_path(self,start, goal):
        start = tuple(start)
        goal = tuple(goal)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        heap = []
        heapq.heappush(heap, (0, [start]))  # Heap element: (cumulative_cost, path)
        visited = {start: 0}

        if start == goal:
            return [start]  # or an empty list, depending on your design

        while heap:
            cost, path = heapq.heappop(heap)
            current = path[-1]
            if current == goal:
                return path, cost
            

            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy
                if 0 <= nx < self.environment["rows"] and 0 <= ny < self.environment["cols"]:
                    step_cost = 1  # Base movement cost
                    penalty = 0
                    if (nx, ny) in self.environment["obstacle_locations"]:
                        penalty = 5  # Additional penalty for stepping into an obstacle
                    new_cost = cost + step_cost + penalty
                    neighbor = (nx, ny)
                    if neighbor not in visited or new_cost < visited[neighbor]:
                        visited[neighbor] = new_cost
                        heapq.heappush(heap, (new_cost, path + [neighbor]))
        return None, float('inf')
   
        
    def compute_goal(self):
        if self.state == "FIND_PICKUP_POINT":
            start = self.location
            goal = self.environment["package_locations"][0]
            self.path, cost = self.find_shortest_path(start, goal)
            print(f" FIND_PICKUP_POINT: Current Location={start}, Package pickup location={goal}, Path={self.path}, Cost={cost}")
            self.current_goal= goal
            self.state = "MOVE_TO_PICKUP"
        elif self.state == "FIND_DROP_POINT":
            start = self.location
            goal = self.environment["drop_off_locations"][0]
            self.path, cost = self.find_shortest_path(start, goal)
            print(f"FIND_DROP_POINT: Package Location={start}, Package drop location={goal}, Path={self.path}, Cost={cost}")
            self.current_goal= goal
            self.state = "MOVE_TO_DROP"




# Example usage:
if __name__ == "__main__":
    #Q1 : Initial Warehouse Configuration:
    warehouse_data = generate_random_warehouse()
    print("Initial Warehouse Configuration\n")
    show_grid(warehouse_data["grid"])
    print("Grid Size:", warehouse_data["rows"], "x", warehouse_data["cols"])
    print("Number of Obstacles:", warehouse_data["num_obstacles"])
    print("Number of Packages:", warehouse_data["num_packages"])
    print("Package Locations:", warehouse_data["package_locations"])
    print("Drop-Off Locations:", warehouse_data["drop_off_locations"])
    print("Obstacle Locations:", warehouse_data["obstacle_locations"])
    print("Robot Start:", warehouse_data["robot_start"])

    agent =GoalBasedAgent(warehouse_data)

    while(agent.state != "COMPLETED"):
        agent.act()

    print(f"All deliveries are completed with total cost {agent.cost} and reward {agent.reward}")
    print(f"Final score is {agent.reward - agent.cost}")