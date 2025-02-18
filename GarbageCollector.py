
import pybullet as p
import time
import pybullet_data
import numpy as np
import CreateObjects
import AstarAlgorithm as Astar
import cv2
import numpy as np
import torch

# Connect to PyBullet
clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

##################################################################################################################
# Loading and setting URDFs

p.loadURDF("plane.urdf", [0, 0, 0.0])
p.setGravity(0, 0, -10)



# Making the maze 
CreateObjects.create_boundaries(physicsClient)

# Placing objects and tray
object_data = CreateObjects.create_garbage_and_tray(physicsClient)


##################################################################################################################
# Importing the husky bot
huskyCenter = [0.0, 0.0, 0.0]#[-9, -2, 0.025]
huskyOrientation = p.getQuaternionFromEuler([0,0,0])
husky = p.loadURDF("husky/husky.urdf", huskyCenter, huskyOrientation)

# Importing kuka arm
kukaCenter = [0.0, 0.0, 0.24023]#[-9, -2, 0.025]
kukaOrientation = p.getQuaternionFromEuler([0,0,0])
scale = 0.4
kukaId = p.loadURDF("kuka_experimental-indigo-devel/kuka_kr210_support/urdf/kr210l150.urdf", kukaCenter, kukaOrientation, globalScaling=scale)

# Setting kuka initially to 0 
curr_joint_value = [0,0,0,0,0,0,0,0,0,0,0]
p.setJointMotorControlArray(kukaId, range(11), p.POSITION_CONTROL, targetPositions=curr_joint_value)

# Putting kuka on husky
cid = p.createConstraint(husky, 1, kukaId, -1, p.JOINT_FIXED, [0, 0, 0], [0.0, 0.0, 0.14023], [0., 0., 0], [0.0, 0.0, 0.0])

# Activating real time simulation
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)

##################################################################################################################
# Inverse kinematics for kuka arm

a1 = 0.75*scale
a2 = 0.35*scale
a3 = 1.25*scale
a4 = 0.054*scale
a5 = 1.5*scale
a6 = 0.303*scale

def get_hypotenuse(a, b):
    return np.sqrt(a*a + b*b)

def get_cosine_law_angle(a, b, c):
    gamma = np.arccos((a*a + b*b - c*c) / (2*a*b))
    return gamma

def griperCenter(px, py, pz, R06): 
    Xc = px - a6*R06[0,2]
    Yc = py - a6*R06[1,2]
    Zc = pz - a6*R06[2,2]
    return Xc, Yc, Zc

def get_first3Angles(Xc, Yc, Zc):
    l = np.sqrt(a4**2 + a5**2)
    phi = np.arctan2(a4, a5)
    
    r1 = get_hypotenuse(Xc, Yc)
    r2 = get_hypotenuse(r1-a2, Zc-a1)

    phi1 = np.arctan2(Zc-a1, r1-a2)
    phi2 = get_cosine_law_angle(r2, a3, l)
    phi3 = get_cosine_law_angle(l, a3, r2)

    q1 = np.arctan2(Yc, Xc)
    q2 = np.pi/2 - phi1 - phi2
    q3 = np.pi/2 -phi3 - phi

    return q1, q2, q3

def get_last3Angles(R36):
    if(R36[2, 2]>=1):
        R36[2, 2] = 1
    elif(R36[2, 2]<=-1):
        R36[2, 2] = -1

    q5 = np.arccos(R36[2, 2])
    q6 = np.arctan2(-R36[2, 1], R36[2, 0])
    q4 = np.arctan2(-R36[1,2], -R36[0,2])

    return q4, q5, q6

def get_kuka_angles(basePosition, baseOrientation, point, orientation):
    a = 0.015772399999437497
    b = 0.009488456000838417
    theta = baseOrientation[2]

    xB = basePosition[0] + a*np.cos(theta) - b*np.sin(theta)
    yB = basePosition[1] + a*np.sin(theta) + b*np.cos(theta)
    zB = basePosition[2] - 0.3587040000000001

    alphaB = baseOrientation[0]
    betaB = baseOrientation[1]
    gamaB = baseOrientation[2]

    xP = point[0]
    yP = point[1]
    zP = point[2]

    alphaP = orientation[0]
    betaP = orientation[1]
    gamaP = orientation[2]

    Hgb = np.array([[np.cos(betaB)*np.cos(gamaB), np.sin(alphaB)*np.sin(betaB)*np.cos(gamaB) - np.sin(gamaB)*np.cos(alphaB), np.sin(alphaB)*np.sin(gamaB) + np.sin(betaB)*np.cos(alphaB)*np.cos(gamaB), xB], 
                    [np.sin(gamaB)*np.cos(betaB), np.sin(alphaB)*np.sin(betaB)*np.sin(gamaB) + np.cos(alphaB)*np.cos(gamaB), -np.sin(alphaB)*np.cos(gamaB) + np.sin(betaB)*np.sin(gamaB)*np.cos(alphaB), yB], 
                    [-np.sin(betaB), np.sin(alphaB)*np.cos(betaB), np.cos(alphaB)*np.cos(betaB), zB],
                    [0, 0, 0, 1]])

    Hgp = np.array([[np.cos(betaP)*np.cos(gamaP), np.sin(alphaP)*np.sin(betaP)*np.cos(gamaP) - np.sin(gamaP)*np.cos(alphaP), np.sin(alphaP)*np.sin(gamaP) + np.sin(betaP)*np.cos(alphaP)*np.cos(gamaP), xP], 
                    [np.sin(gamaP)*np.cos(betaP), np.sin(alphaP)*np.sin(betaP)*np.sin(gamaP) + np.cos(alphaP)*np.cos(gamaP), -np.sin(alphaP)*np.cos(gamaP) + np.sin(betaP)*np.sin(gamaP)*np.cos(alphaP), yP], 
                    [-np.sin(betaP), np.sin(alphaP)*np.cos(betaP), np.cos(alphaP)*np.cos(betaP), zP],
                    [0, 0, 0, 1]])

    IHgb = np.linalg.inv(Hgb)
    Hbp = np.dot(IHgb, Hgp)

    R6a = Hbp[:3, :3]
    R6b = np.array([[0, 0, 1.0], 
                    [0, -1.0, 0], 
                    [1.0, 0, 0]])

    R06 = np.dot(R6a, R6b)
    [Px, Py, Pz] = Hbp[:3, 3]

    Xc, Yc, Zc = griperCenter(Px, Py, Pz, R06)
    q1, q2, q3 = get_first3Angles(Xc, Yc, Zc)
    
    R03 = [[np.sin(q2)*np.cos(q1)*np.cos(q3) + np.sin(q3)*np.cos(q1)*np.cos(q2), np.sin(q1), -np.sin(q2)*np.sin(q3)*np.cos(q1) + np.cos(q1)*np.cos(q2)*np.cos(q3)], 
           [np.sin(q1)*np.sin(q2)*np.cos(q3) + np.sin(q1)*np.sin(q3)*np.cos(q2), -np.cos(q1), -np.sin(q1)*np.sin(q2)*np.sin(q3) + np.sin(q1)*np.cos(q2)*np.cos(q3)], 
           [-np.sin(q2)*np.sin(q3) + np.cos(q2)*np.cos(q3), 0, -np.sin(q2)*np.cos(q3) - np.sin(q3)*np.cos(q2)]]

    IR03 = np.transpose(R03)
    R36 = np.dot(IR03, R06)
    q4, q5, q6 = get_last3Angles(R36)

    return q1, q2, q3, q4, q5, q6

def get_point_parameters(curr_joint_value, final_joint_value, t):
    """
    Interpolates between current and final joint values with smooth motion
    """
    q1 = curr_joint_value[0] + (final_joint_value[0] - curr_joint_value[0])*t
    q2 = curr_joint_value[1] + (final_joint_value[1] - curr_joint_value[1])*t
    q3 = curr_joint_value[2] + (final_joint_value[2] - curr_joint_value[2])*t
    q4 = curr_joint_value[3] + (final_joint_value[3] - curr_joint_value[3])*t
    q5 = curr_joint_value[4] + (final_joint_value[4] - curr_joint_value[4])*t
    q6 = curr_joint_value[5] + (final_joint_value[5] - curr_joint_value[5])*t
    
    return q1, q2, q3, q4, q5, q6

##################################################################################################################
# Motion control functions
def display_camera_feed(kukaId, target_pos):
    """
    Display camera feed from end effector
    """
    camera_params = {
        'width': 320,
        'height': 240,
        'fov': 60,
        'near': 0.01,
        'far': 100.0,
        'aspect': 320/240
    }
    
    # Get end effector position
    ee_state = p.getLinkState(kukaId, 6)
    camera_pos = ee_state[0]
    
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_pos,
        cameraTargetPosition=target_pos,
        cameraUpVector=[0, 0, 1]
    )

    proj_matrix = p.computeProjectionMatrixFOV(
        fov=camera_params['fov'],
        aspect=camera_params['aspect'],
        nearVal=camera_params['near'],
        farVal=camera_params['far']
    )

    # Get camera image
    _, _, rgb_img, depth_img, seg_img = p.getCameraImage(
        width=camera_params['width'],
        height=camera_params['height'],
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )
    
    # Convert to BGR for OpenCV
    rgb_array = np.array(rgb_img, dtype=np.uint8)
    rgb_array = np.reshape(rgb_array, (camera_params['height'], camera_params['width'], 4))
    bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGBA2BGR)
    
    # Display image
    cv2.imshow('End Effector Camera', bgr_array)
    cv2.waitKey(1)

    return rgb_array, depth_img, seg_img, camera_params

# Initialize YOLO model (add this after imports)
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.conf = 0.25  # Confidence threshold
model.iou = 0.45   # NMS IOU threshold
model.classes = None  # Detect all classes

def detect_object(rgb_img):
    """
    Detect objects in the camera feed using YOLO
    """
    try:
        # Ensure proper image format
        if len(rgb_img.shape) == 4:  # If RGBA
            rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGBA2RGB)
        elif len(rgb_img.shape) == 3 and rgb_img.shape[2] == 4:  # If RGBA
            rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGBA2RGB)
        
        # Run inference
        with torch.no_grad():  # Disable gradient calculation for faster inference
            results = model(rgb_img)
        
        # Get detections
        detections = results.pandas().xyxy[0]
        
        # Define relevant classes for our task
        relevant_classes = [
            'bottle', 'cup', 'bowl', 'orange', 'apple', 'banana',
            'cell phone', 'mouse', 'remote', 'keyboard', 'book',
            'vase', 'scissors', 'teddy bear', 'box', 'can', 'container',
            'toy', 'object', 'cylinder', 'food'
        ]
        
        # Debug output
        if not detections.empty:
            print("Detected objects:")
            for idx, detection in detections.iterrows():
                print(f"- {detection['name']}: {detection['confidence']:.2f}")
        
        # Filter for relevant classes
        filtered_detections = detections[detections['name'].isin(relevant_classes)]
        
        if not filtered_detections.empty:
            # Get best detection
            best_detection = filtered_detections.iloc[filtered_detections['confidence'].argmax()]
            
            # Return detection info
            return {
                'class': best_detection['name'],
                'confidence': best_detection['confidence'],
                'bbox': [
                    best_detection['xmin'],
                    best_detection['ymin'],
                    best_detection['xmax'],
                    best_detection['ymax']
                ]
            }
        
        # If no relevant objects found, try any detected object
        elif not detections.empty:
            best_detection = detections.iloc[detections['confidence'].argmax()]
            print(f"Found non-listed object: {best_detection['name']}")
            return {
                'class': best_detection['name'],
                'confidence': best_detection['confidence'],
                'bbox': [
                    best_detection['xmin'],
                    best_detection['ymin'],
                    best_detection['xmax'],
                    best_detection['ymax']
                ]
            }
        
        return None

    except Exception as e:
        print(f"Error in object detection: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

def speed_for_rotation(rotation, currOrientation):
    kp = 7
    Vmax = 5
    if(abs(currOrientation - rotation) < np.pi):
        v = kp*(currOrientation - rotation)
    else:
        if(currOrientation > 0):
            v = kp*(currOrientation - (rotation + 2*np.pi))
        else:
            v = kp*(currOrientation - (rotation - 2*np.pi))

    if(v>Vmax):
        v = Vmax
    elif(v<-Vmax):
        v = -Vmax

    return v

def speed_for_forward(delX, delY):
    kp = 10
    Vmax = 5
    v = kp*(abs(delX)+abs(delY))/2

    if(v>Vmax):
        v = Vmax
    elif(v<-Vmax):
        v = -Vmax

    return v

def get_target_parameters(x, y, z):
    currPosition = p.getLinkStates(husky, [0])[0][0]
    currOrientation = p.getEulerFromQuaternion(p.getLinkStates(husky, [0])[0][1])

    deltaX = x - currPosition[0]
    deltaY = y - currPosition[1]

    rotation = [0, 0, np.arctan2(deltaY, deltaX)]
    return deltaX, deltaY, rotation, currOrientation, currPosition

def move_husky_to_point(x, y, z):
    wheels = [2, 3, 4, 5]
    wheelVelocities = [0, 0, 0, 0]
    wheelDeltasTurn = [1, -1, 1, -1]
    wheelDeltasFwd = [1, 1, 1, 1]

    deltaX, deltaY, rotation, currOrientation, currPosition = get_target_parameters(x, y, z)

    while abs(rotation[2]-currOrientation[2])>= 0.005:
        deltaX, deltaY, rotation, currOrientation, currPosition = get_target_parameters(x, y, z)
        wheelVelocities = [0, 0, 0, 0]
        vr = speed_for_rotation(rotation[2], currOrientation[2])
        
        for i in range(len(wheels)):
            wheelVelocities[i] = wheelVelocities[i] + vr * wheelDeltasTurn[i]
            p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)

    while abs(deltaX)>= 0.05 or abs(deltaY)>=0.05:
        deltaX, deltaY, rotation, currOrientation, currPosition = get_target_parameters(x, y, z)
        wheelVelocities = [0, 0, 0, 0]
        
        vr = speed_for_rotation(rotation[2], currOrientation[2])
        vf = speed_for_forward(deltaX, deltaY)

        for i in range(len(wheels)):
            wheelVelocities[i] = wheelVelocities[i] + vr * wheelDeltasTurn[i] + vf * wheelDeltasFwd[i]
            p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)

    wheelVelocities = [0, 0, 0, 0]
    for i in range(len(wheels)):
        p.setJointMotorControl2(husky,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)

def move_endeffector_to_point(finalPoint, finalOrientation):
    """
    Enhanced end-effector movement with smoother motion
    """
    kukaBasePosition = p.getLinkStates(kukaId, [0])[0][0]
    kukaBaseOrientation = p.getEulerFromQuaternion(p.getLinkStates(husky, [0])[0][1])
    final_joint_value = get_kuka_angles(kukaBasePosition, kukaBaseOrientation, finalPoint, finalOrientation)
    
    t = 0
    while t <= 1:
        q1, q2, q3, q4, q5, q6 = get_point_parameters(curr_joint_value, final_joint_value, t)
        p.setJointMotorControlArray(kukaId, range(11), p.POSITION_CONTROL, 
                                  targetPositions=[q1, q2, q3, q4, q5, q6, 
                                                 curr_joint_value[6], curr_joint_value[7], 
                                                 curr_joint_value[8], curr_joint_value[9], 
                                                 curr_joint_value[10]],
                                  forces=[500.0] * 11)  # Added force control
        t += .001  # Slower movement for more stability
        time.sleep(0.0001)  # Small delay for smoother motion

    curr_joint_value[0:6] = final_joint_value



def calculate_position_for_husky(x2, y2, z2):
    x1, y1, z1 = p.getLinkStates(husky, [0])[0][0]
    t = 0.7 / np.sqrt((x1-x2)**2 + (y1-y2)**2)
    x = x2 + t*(x1-x2)
    y = y2 + t*(y1-y2)
    return x, y, z2

def convertCoordinates(x,y):
    x = x/4-11
    y = y/4-11
    return x,y
'''
def hold(flag, kukaId):
    """
    Enhanced gripper control with stronger grip and better release
    """
    if flag:  # Grasp
        # First open wide
        p.setJointMotorControlArray(
            kukaId, 
            [7, 8], 
            p.POSITION_CONTROL, 
            targetPositions=[0.4, 0.4],  # Wider opening
            forces=[1000, 1000]  # Strong force for quick opening
        )
        time.sleep(0.5)
        
        # Then close with very high force for tight grip
        p.setJointMotorControlArray(
            kukaId, 
            [7, 8], 
            p.POSITION_CONTROL, 
            targetPositions=[0.0, 0.0],  # Complete closure
            forces=[2000, 2000]  # Very high force for tight grip
        )
        time.sleep(0.8)  # Longer time to establish firm grip
        
    else:  # Release
        # Quick and wide opening
        p.setJointMotorControlArray(
            kukaId, 
            [7, 8], 
            p.POSITION_CONTROL, 
            targetPositions=[0.4, 0.4],  # Maximum opening
            forces=[1000, 1000]  # High force for quick release
        )
        time.sleep(0.3)
'''
def hold(flag, kukaId):
    """
    Enhanced gripper control with stronger grip
    """
    if flag:  # Grasp
        # First open wide
        p.setJointMotorControlArray(
            kukaId, 
            [7, 8], 
            p.POSITION_CONTROL, 
            targetPositions=[0.4, 0.4],  # Wider opening
            forces=[1000, 1000]  # Strong force for quick opening
        )
        time.sleep(0.3)
        
        # Then close with very high force for tight grip
        p.setJointMotorControlArray(
            kukaId, 
            [7, 8], 
            p.POSITION_CONTROL, 
            targetPositions=[0.0, 0.0],  # Complete closure
            forces=[2000, 2000]  # Very high force for tight grip
        )
        time.sleep(0.5)  # Allow grip to establish
        
    else:  # Release
        p.setJointMotorControlArray(
            kukaId, 
            [7, 8], 
            p.POSITION_CONTROL, 
            targetPositions=[0.4, 0.4],  # Maximum opening
            forces=[1000, 1000]
        )
        time.sleep(0.2)
'''       
def pick_cube_from(position, object_id):

    [x2, y2, z2] = position
    x1, y1, z1 = p.getLinkStates(husky, [0])[0][0]
    path = Astar.calculateShortestPath([x1, y1], [x2, y2])
    
    # Open gripper fully before starting movement
    hold(0,kukaId)
    time.sleep(0.3)

    # Follow path to object
    for i in range(len(path) - 1):
        x3, y3 = path[i]
        x3, y3 = convertCoordinates(x3, y3)
        if np.sqrt((x2 - x3)**2 + (y2 - y3)**2) <= 0.8:
            break
        move_husky_to_point(x3, y3, z2)

    # Calculate final position near object
    x, y, z = calculate_position_for_husky(x2, y2, z2)
    move_husky_to_point(x, y, z)

    initialOrientation = p.getLinkStates(kukaId, [6])[0][1]
    initialOrientation = p.getEulerFromQuaternion(initialOrientation)
    time.sleep(0.5)  # Allow robot to stabilize

    # Move to a safe height above the object
    move_endeffector_to_point([x2, y2, 0.25], [0, np.pi/2, 0])  # Safe height above object
    time.sleep(0.3)

    # **Ensure gripper is open before descending**
    hold(0,kukaId)  # Explicitly open the gripper
    time.sleep(0.3)  # Delay to ensure gripper is fully open

    # Lower closer to the object for grasping
    move_endeffector_to_point([x2, y2, 0.08], [0, np.pi/2, 0])  # Lower to grasping height
    time.sleep(0.5)  # Stabilize at grasping position

    # Now close gripper to grasp
    hold(1,kukaId)
    time.sleep(0.8)  # Give time for grip to establish

    # Lift object gradually
    move_endeffector_to_point([x2, y2, 0.2], [0, np.pi/2, 0])  # Initial lift
    time.sleep(0.2)
    move_endeffector_to_point([x2, y2, 0.4], [0, np.pi/2, 0])  # Higher lift
    time.sleep(0.3)
    
    # Return to safe height
    move_endeffector_to_point([x2, y2, 1], initialOrientation)
    time.sleep(0.3)

    return True
'''
def display_detection_text(detection):
    """
    Display detection results in simulation window
    """
    text_pos = [0, 0, 1.5]  # Position above the scene
    text_color = [0, 1, 0]  # Green color for detection
    
    if detection:
        text = f"Detected: {detection['class']} ({detection['confidence']:.2f})"
    else:
        text = "Searching for object..."
        text_color = [1, 0, 0]  # Red color while searching
    
    # Remove previous text to avoid clutter
    p.removeAllUserDebugItems()
    
    # Add new text
    p.addUserDebugText(
        text,
        text_pos,
        text_color,
        textSize=1.5,
        lifeTime=0.5  # Short lifetime for continuous updates
    )
'''
def pick_cube_from(position, object_id):
    """
    Enhanced picking function with camera integration
    """
    try:
        # Setup camera window
        cv2.namedWindow('End Effector Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('End Effector Camera', 320, 240)
        
        # Increase friction for objects
        p.changeDynamics(object_id, -1, 
                        lateralFriction=1.0,
                        spinningFriction=0.1,
                        rollingFriction=0.1,
                        restitution=0.1,
                        contactStiffness=10000,
                        contactDamping=1)
        
        [x2, y2, z2] = position
        print(f"Moving to pickup position: {position}")

        # Move Husky to position with A*
        x1, y1, z1 = p.getLinkStates(husky, [0])[0][0]
        path = Astar.calculateShortestPath([x1,y1],[x2,y2])
        
        for i in range(len(path)-1):
            x3,y3 = path[i]
            x3,y3 = convertCoordinates(x3,y3)
            if(np.sqrt((x2-x3)**2 + (y2-y3)**2)<=0.8):
                break
            move_husky_to_point(x3, y3, z2)
            display_camera_feed(kukaId, [x2, y2, z2])
        
        x, y, z = calculate_position_for_husky(x2, y2, z2)
        move_husky_to_point(x, y, z)

        # Pre-grasp preparation
        initialOrientation = p.getEulerFromQuaternion(p.getLinkStates(kukaId, [6])[0][1])
        
        # Open gripper fully before approach
        hold(0,kukaId)

        # Detection and approach sequence
        approach_heights = [0.3, 0.15]
        object_detected = False
        detection = None
        
        for height in approach_heights:
            move_endeffector_to_point([x2, y2, height], [0, np.pi/2, 0])
            time.sleep(0.5)
            
            rgb_img, _, _, _ = display_camera_feed(kukaId, [x2, y2, z2])
            current_detection = detect_object(rgb_img)
            
            if current_detection:
                detection = current_detection
                object_detected = True
                print(f"Object detected: {detection['class']} with confidence {detection['confidence']:.2f}")
                
                # Position adjustment based on detection
                bbox = detection['bbox']
                center_x = (bbox[0] + bbox[2]) / 2
                center_y = (bbox[1] + bbox[3]) / 2
                x_offset = (center_x - 160) * 0.0002
                y_offset = (center_y - 120) * 0.0002
                x2 += x_offset
                y2 += y_offset
                break
        
        # Final approach and grasp
        move_endeffector_to_point([x2, y2, 0.08], [0, np.pi/2, 0])
        time.sleep(1.0)
        display_camera_feed(kukaId, [x2, y2, z2])
        
        # Close gripper with more force
        p.setJointMotorControl2(kukaId, 7, p.POSITION_CONTROL, targetPosition=0.0, force=200)
        p.setJointMotorControl2(kukaId, 8, p.POSITION_CONTROL, targetPosition=0.0, force=200)
        time.sleep(0.3)

        # Check if grasp was successful
        object_pos = p.getBasePositionAndOrientation(object_id)[0]
        gripper_pos = p.getLinkState(kukaId, 6)[0]
        
        if abs(object_pos[2] - gripper_pos[2]) < 0.1:
            # Lift sequence with camera updates
            lift_heights = [0.2, 0.35]
            for height in lift_heights:
                move_endeffector_to_point([x2, y2, height], [0, np.pi/2, 0])
                time.sleep(0.1)
                display_camera_feed(kukaId, [x2, y2, z2])
            return True
        else:
            print("Grasp failed, releasing gripper")
            hold(0,kukaId)
            move_endeffector_to_point([x2, y2, 0.4], [0, np.pi/2, 0])
            return False
            
    finally:
        cv2.destroyAllWindows()
'''
def pick_cube_from(position, object_id):
    """
    Enhanced picking function with detection text display
    """
    try:
        # Setup camera window
        cv2.namedWindow('End Effector Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('End Effector Camera', 320, 240)
        
        # Increase friction for objects
        p.changeDynamics(object_id, -1, 
                        lateralFriction=1.0,
                        spinningFriction=0.1,
                        rollingFriction=0.1,
                        restitution=0.1,
                        contactStiffness=10000,
                        contactDamping=1)
        
        [x2, y2, z2] = position
        print(f"Moving to pickup position: {position}")

        # Move Husky to position with A*
        x1, y1, z1 = p.getLinkStates(husky, [0])[0][0]
        path = Astar.calculateShortestPath([x1,y1],[x2,y2])
        
        for i in range(len(path)-1):
            x3,y3 = path[i]
            x3,y3 = convertCoordinates(x3,y3)
            if(np.sqrt((x2-x3)**2 + (y2-y3)**2)<=0.8):
                break
            move_husky_to_point(x3, y3, z2)
            display_camera_feed(kukaId, [x2, y2, z2])
        
        x, y, z = calculate_position_for_husky(x2, y2, z2)
        move_husky_to_point(x, y, z)

        # Pre-grasp preparation
        initialOrientation = p.getEulerFromQuaternion(p.getLinkStates(kukaId, [6])[0][1])
        
        # Open gripper fully before approach
        hold(0,kukaId)

        # Detection and approach sequence
        approach_heights = [0.3, 0.15]
        object_detected = False
        detection = None
        
        for height in approach_heights:
            move_endeffector_to_point([x2, y2, height], [0, np.pi/2, 0])
            time.sleep(0.5)
            
            rgb_img, _, _, _ = display_camera_feed(kukaId, [x2, y2, z2])
            current_detection = detect_object(rgb_img)
            
            # Display detection status
            display_detection_text(current_detection)
            
            if current_detection:
                detection = current_detection
                object_detected = True
                print(f"Object detected: {detection['class']} with confidence {detection['confidence']:.2f}")
                
                # Position adjustment based on detection
                bbox = detection['bbox']
                center_x = (bbox[0] + bbox[2]) / 2
                center_y = (bbox[1] + bbox[3]) / 2
                x_offset = (center_x - 160) * 0.0002
                y_offset = (center_y - 120) * 0.0002
                x2 += x_offset
                y2 += y_offset
                break
        
        # Final approach and grasp
        move_endeffector_to_point([x2, y2, 0.08], [0, np.pi/2, 0])
        time.sleep(1.0)
        display_camera_feed(kukaId, [x2, y2, z2])
        
        # Close gripper with more force
        p.setJointMotorControl2(kukaId, 7, p.POSITION_CONTROL, targetPosition=0.0, force=200)
        p.setJointMotorControl2(kukaId, 8, p.POSITION_CONTROL, targetPosition=0.0, force=200)
        time.sleep(0.3)

        # Check if grasp was successful
        object_pos = p.getBasePositionAndOrientation(object_id)[0]
        gripper_pos = p.getLinkState(kukaId, 6)[0]
        
        if abs(object_pos[2] - gripper_pos[2]) < 0.1:
            # Lift sequence with camera updates
            lift_heights = [0.2, 0.35]
            for height in lift_heights:
                move_endeffector_to_point([x2, y2, height], [0, np.pi/2, 0])
                time.sleep(0.1)
                display_camera_feed(kukaId, [x2, y2, z2])
            return True
        else:
            print("Grasp failed, releasing gripper")
            hold(0,kukaId)
            move_endeffector_to_point([x2, y2, 0.4], [0, np.pi/2, 0])
            return False
            
    finally:
        cv2.destroyAllWindows()

'''
def place_cube_to(trayPosition, kukaId, object_id):
    """
    Enhanced placing function with forceful drop
    """
    [x2, y2, z2] = trayPosition
    x1, y1, z1 = p.getLinkStates(husky, [0])[0][0]
    path = Astar.calculateShortestPath([x1,y1],[x2,y2])
    
    # Move to tray position
    for i in range(len(path)-1):
        x3,y3 = path[i]
        x3,y3 = convertCoordinates(x3,y3)
        if(np.sqrt((x2-x3)**2 + (y2-y3)**2)<=0.8):
            break
        move_husky_to_point(x3, y3, z2)
    
    # Get to final position
    x, y, z = calculate_position_for_husky(x2, y2, z2)
    move_husky_to_point(x, y, z)
    time.sleep(0.5)

    # Move above tray
    move_endeffector_to_point([x2, y2, 0.5], [0, np.pi/2, 0])
    time.sleep(0.5)

    # Prepare object for dropping
    p.changeDynamics(
        object_id, 
        -1,
        mass=2.0,  # Increase mass for better dropping
        lateralFriction=0.1,  # Reduce friction
        spinningFriction=0.1,
        rollingFriction=0.1,
        restitution=0.1,  # Some bounce but not too much
        contactStiffness=10000,
        contactDamping=1
    )

    # Quick release with force
    hold(0, kukaId)  # Open gripper wide
    time.sleep(0.2)
    
    # Apply downward force to object
    p.applyExternalForce(
        object_id,
        -1,
        forceObj=[0, 0, -500],  # Strong downward force
        posObj=[x2, y2, z2 + 0.1],
        flags=p.WORLD_FRAME
    )
    
    time.sleep(0.5)  # Wait for object to fall
    
    # Return to safe position
    move_endeffector_to_point([x2, y2, 1], [0, np.pi/2, 0])
    return True
'''

def calculate_safe_approach_to_tray(current_pos, tray_position):
    """
    Calculate a safe approach path to tray avoiding other tray
    """
    [x2, y2, z2] = tray_position
    [x1, y1, z1] = current_pos
    
    # Define tray positions
    tray1_pos = [1, -5.5, 0.025]
    tray2_pos = [-1, -5.5, 0.025]
    
    # Determine which tray we're approaching and which to avoid
    target_tray = 1 if abs(x2 - tray1_pos[0]) < 0.1 else 2
    
    # Calculate intermediate points for safe approach
    if target_tray == 1:
        # Approaching tray 1, avoid tray 2
        safe_y = -4.0  # Safe Y coordinate before final approach
        intermediate_points = [
            [x1, y1, z1],  # Start
            [x2, safe_y, z1],  # Safe point above target tray
            [x2, y2, z2]   # Final tray position
        ]
    else:
        # Approaching tray 2, avoid tray 1
        safe_y = -4.0  # Safe Y coordinate before final approach
        intermediate_points = [
            [x1, y1, z1],  # Start
            [x2, safe_y, z1],  # Safe point above target tray
            [x2, y2, z2]   # Final tray position
        ]
    
    return intermediate_points

def place_cube_to(trayPosition, kukaId, object_id):
    """
    Enhanced placing function with tray avoidance
    """
    try:
        [x2, y2, z2] = trayPosition
        current_pos = p.getLinkStates(husky, [0])[0][0]
        
        # Calculate safe approach path
        approach_points = calculate_safe_approach_to_tray(current_pos, trayPosition)
        
        # Follow safe approach path
        for point in approach_points[:-1]:  # Don't move to final point yet
            x, y, z = calculate_position_for_husky(point[0], point[1], point[2])
            move_husky_to_point(x, y, z)
            time.sleep(0.3)  # Allow for stable positioning
        
        # Final approach to tray
        x, y, z = calculate_position_for_husky(x2, y2, z2)
        move_husky_to_point(x, y, z)
        time.sleep(0.5)  # Ensure stability before placing

        # Prepare object for dropping
        p.changeDynamics(
            object_id, 
            -1,
            mass=2.0,
            lateralFriction=0.1,
            spinningFriction=0.1,
            rollingFriction=0.1,
            restitution=0.1,
            contactStiffness=10000,
            contactDamping=1
        )

        # Position end effector above tray
        move_endeffector_to_point([x2, y2, 0.5], [0, np.pi/2, 0])
        time.sleep(0.3)
        
        # Lower to dropping height
        move_endeffector_to_point([x2, y2, 0.3], [0, np.pi/2, 0])
        time.sleep(0.2)

        # Release object
        hold(0, kukaId)
        time.sleep(0.3)
        
        # Apply downward force for controlled drop
        p.applyExternalForce(
            object_id,
            -1,
            forceObj=[0, 0, -500],
            posObj=[x2, y2, z2 + 0.1],
            flags=p.WORLD_FRAME
        )
        
        time.sleep(0.5)  # Wait for object to settle
        
        # Retract end effector
        move_endeffector_to_point([x2, y2, 0.5], [0, np.pi/2, 0])
        time.sleep(0.2)
        move_endeffector_to_point([x2, y2, 1.0], [0, np.pi/2, 0])
        
        return True

    except Exception as e:
        print(f"Error during placement: {str(e)}")
        return False

##################################################################################################################


# Main execution

def main():
    trayPosition1 = [1, -5.5, 0.025]
    trayPosition2 = [-1, -5.5, 0.025]
    start = time.time()

    # Pick and place each object

    # Pick and place strawberry
    print("\nAttempting to pick strawberry")
 
    pick_cube_from(object_data.get('soupcan')['position'], object_data.get('soupcan')['id'])
    place_cube_to(trayPosition1,kukaId,object_data.get('soupcan')['id'])

 

    # Pick and place banana
    print("\nAttempting to pick banana")
    pick_cube_from(object_data.get('orange2')['position'], object_data.get('orange2')['id'])
    place_cube_to(trayPosition2,kukaId,object_data.get('orange2')['id'])

    # Pick and place banana
    print("\nAttempting to pick tennisball")
    pick_cube_from(object_data.get('tennisball')['position'], object_data.get('tennisball')['id'])
    place_cube_to(trayPosition1,kukaId,object_data.get('tennisball')['id'])
    
    # Return to home position and wave
    print("\nReturning to home position")
    #move_husky_to_point_in_maze(0, 0, 0)

    end = time.time()
    print("\nTotal time taken:", end-start)

if __name__ == "__main__":
    try:
        main()
        # time.sleep(10)
    finally:
        p.disconnect()

