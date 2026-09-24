from controller import Supervisor

robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

maze = [
    [1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 0, 1, 1, 1]
]

CELL_SIZE = 0.30
WALL_THICKNESS = 0.30
WALL_HEIGHT = 0.50

root = robot.getRoot()
children = root.getField("children")


def create_wall(row, col):
   x = row * CELL_SIZE
   y = col * CELL_SIZE
   z = WALL_HEIGHT / 2

   wall = f"""
    Solid {{
        translation {x} {y} {z}
        children [
            Shape {{
                geometry Box {{
                    size {CELL_SIZE} {WALL_THICKNESS} {WALL_HEIGHT}
                }}
            }}
        ]
        boundingObject Box {{
            size {CELL_SIZE} {WALL_THICKNESS} {WALL_HEIGHT}
        }}
    }}
    """

    
   children.importMFNodeFromString(-1, wall)
    
for row in range(len(maze)):
    for col in range(len(maze[row])):
        if maze[row][col] == 1:
            create_wall(row, col)
            
    
while robot.step(timestep) != -1:
    pass