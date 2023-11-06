import time
from flask import Flask, request, jsonify
from flask_cors import CORS


from algo.algo import MazeSolver
from helper import command_generator

app = Flask(__name__)
CORS(app)
#model = load_model()
model = None
@app.route('/status', methods=['GET'])
def status():
    """
    
    """
    print("::::Got the obstacles from RPI::::\n")
    
    return jsonify({"result": "ok"})


@app.route('/path', methods=['POST'])
def path_finding():
    """
    
    """
    # Get the json data from the request
    content = request.json

    print("\n::::Got the obstacles from RPI::::\n")
    print(f"Recieved Content from RPI is:: {content}\n")
    # Create an instance of the maze solver class
    



    # Get the obstacles, big_turn, retrying, robot_x, robot_y, and robot_direction from the json data
    obstacles = content['obstacles']
    # big_turn = int(content['big_turn'])
    retrying = content['retrying']
    robot_x, robot_y = content['robot_x'], content['robot_y']
    robot_direction = int(content['robot_dir'])
    
    print("\n:::CREATING MAZE ON LAPTOP NOW:::")
    # Initialize MazeSolver object with robot size of 20x20, bottom left corner of robot at (1,1), facing north, and whether to use a big turn or not.
    maze_solver = MazeSolver(20, 20, robot_x, robot_y, robot_direction, big_turn=None)
    
    # Add each obstacle into the MazeSolver. Each obstacle is defined by its x,y positions, its direction, and its id
    for ob in obstacles:
        maze_solver.add_obstacle(ob['x'], ob['y'], ob['d'], ob['id'])

    print("::::Running through A*Star Algo::::\n")
    start = time.time()
    # Get shortest path
    optimal_path, distance = maze_solver.get_optimal_order_dp(retrying=retrying)
    end=time.time()
    
    print("::::::::PATH FOUND:::::::::\n\n")
    print("Printing the path stats now...")

    time.sleep(0.09)
    print(f"Time taken to find shortest path using A* search: {end - start}s")
    print(f"Distance to travel: {distance} units")
    
    # Based on the shortest path, generate commands for the robot
    commands = command_generator(optimal_path, obstacles)

    # Get the starting location and add it to path_results
    path_results = [optimal_path[0].get_dict()]
    # Process each command individually and append the location the robot should be after executing that command to path_results
    i = 0
    for command in commands:
        if command.startswith("SNAP"):
            continue
        if command.startswith("FIN"):
            continue
        elif command.startswith("FW") or command.startswith("FS"):
            i += int(command[2:]) // 10
        elif command.startswith("BW") or command.startswith("BS"):
            i += int(command[2:]) // 10
        else:
            i += 1
        path_results.append(optimal_path[i].get_dict())

    
    #print(f"\nFinal distance is: {distance}")
    print(f"Path Results are: {path_results}\n")
    print(f"\nPath Commands are: {commands}\n")

    print(":::Sending back to RPI:::")


    return jsonify({
        "data": {
            'distance': distance,
            'path': path_results,
            'commands': commands
        },
        "error": None
    })


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
