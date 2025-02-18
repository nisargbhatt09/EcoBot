import pybullet as p

def create_boundaries(physicsClient):
   
    # Creating walls
    p.loadURDF("cubes/urdf/6y_segment.urdf", basePosition=[4, 6, 0.1], 
               useFixedBase=1, physicsClientId=physicsClient)
    p.loadURDF("cubes/urdf/6y_segment.urdf", basePosition=[4, -6, 0.1], 
               useFixedBase=1, physicsClientId=physicsClient)
    p.loadURDF("cubes/urdf/6y_segment.urdf", basePosition=[-4, 6, 0.1], 
               useFixedBase=1, physicsClientId=physicsClient)
    p.loadURDF("cubes/urdf/6y_segment.urdf", basePosition=[-4, -6, 0.1], 
               useFixedBase=1, physicsClientId=physicsClient)
    p.loadURDF("cubes/urdf/4x_segment.urdf", basePosition=[0, 6, 0.1], 
               useFixedBase=1, physicsClientId=physicsClient)
    p.loadURDF("cubes/urdf/4x_segment.urdf", basePosition=[0, -6, 0.1], 
               useFixedBase=1, physicsClientId=physicsClient)

def create_garbage_and_tray(physicsClient):
    # Define positions for objects with their IDs
    object_data = {
        "soupcan": {
            "position": [1, 1, 0.025],
            #"position": [2, -1.5, -0.025],
            "urdf": "cubes/urdf/YcbTomatoSoupCan/model.urdf",
            "id": None
        },
        "orange2": {
            "position": [2, 5, -0.025],
            #"position": [2, -1.5, -0.025],
            "urdf": "cubes/urdf/plastic_orange/model.urdf",
            "id": None
        },
        "tennisball": {
            "position": [-5, 0, 0.025],
            "urdf": "cubes/urdf/YcbTennisBall/model.urdf",
            "id": None
        },
        # Uncomment these blocks if needed
        # "orange": {
        #     "position": [-2, 5, 0.025],
        #     "urdf": "cubes/urdf/plastic_orange/model.urdf",
        #     "id": None
        # },
        # "chip": {
        #     "position": [8, -7, 0.025],
        #     "urdf": "cubes/urdf/potato_chip_1/model.urdf",
        #     "id": None
        # },
        # "fifth_object": {
        #     "position": [-3, 7, 0.025],
        #     "urdf": "path/to/your/fifth.urdf",
        #     "id": None
        # },
        # "sixth_object": {
        #     "position": [3, -7, 0.025],
        #     "urdf": "path/to/your/sixth.urdf",
        #     "id": None
        # }
    }
    
    # Tray position
    tray_position1 = [1, -5.5, 0.00025]
    tray_position2 = [-1, -5.5, 0.00025]
    # Place the tray
    tray1 = p.loadURDF(
        "cubes/urdf/traybox1.urdf", 
        basePosition=tray_position1, 
        useFixedBase=1, 
        physicsClientId=physicsClient
    )
    tray2 = p.loadURDF(
        "cubes/urdf/traybox2.urdf", 
        basePosition=tray_position2, 
        useFixedBase=1, 
        physicsClientId=physicsClient
    )
    # r: 186, g: 184, b: 108
    p.changeVisualShape(tray1, -1, rgbaColor=[0.186, 0.184, 0.108, 0.5])
    p.changeVisualShape(tray2, -1, rgbaColor=[0.8, 0.8, 0.8, 1])
    # Load each object and store its ID
    for obj_name, obj_data in object_data.items():
        obj_id = p.loadURDF(
            obj_data["urdf"], 
            basePosition=obj_data["position"],
            globalScaling=1.0, 
            physicsClientId=physicsClient
        )
        obj_data["id"] = obj_id
        print(f"Spawned {obj_name} with ID {obj_id} at position {obj_data['position']}")  # Debug print
    
    return object_data
