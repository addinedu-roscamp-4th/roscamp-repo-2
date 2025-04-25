# 송신 클래스
## 알바봇 정보
{
    msg_type: "Albabot",
    robot_id: int,
    battery_level: int,
    pinky_x: float,
    pinky_y: float,
    pinky_z: float,
    pinky_roll: float,
    pinky_pitch: float,
    pinky_yaw: float,
    global_x: float,
    global_y: float,
    global_z: float,
    global_roll: float,
    global_pitch: float,
    global_yaw: float,
    world_x: float,
    world_y: float,
    world_z: float,
    world_roll: float,
    world_pitch: float,
    world_yaw: float,
}
# 수신 클래스
## Albabot
{
    robot_id: robot_id,
    battery_level: battery_level,
    status: status(recent),
    timestamp: now(),
}

## Pose6D
{
    entity_id: robot_id,
    entity_type: "PINKY",
    timestamp: now(),
    x: pinky_x,
    y: pinky_y,
    z: pinky_z,
    roll: pinky_roll,
    pitch: pinky_pitch,
    yaw: pinky_yaw,
}
{
    entity_id: robot_id,
    entity_type: "GLOBAL",
    timestamp: now(),
    x: global_x,
    y: global_y,
    z: global_z,
    roll: global_roll,
    pitch: global_pitch,
    yaw: global_yaw,
}
{
    entity_id: robot_id,
    entity_type: "WORLD",
    timestamp: now(),
    x: world_x,
    y: world_y,
    z: world_z,
    roll: world_roll,
    pitch: world_pitch,
    yaw: world_yaw,
}

# 송신 클래스
## 로봇팔 정보
{
    msg_type: "Cookbot",
    robot_id: int,
    angle_1: float,
    angle_2: float,
    angle_3: float,
    angle_4: float,
    angle_5: float,
    angle_6: float,
    endpoint_x: float,
    endpoint_y: float,
    endpoint_z: float,
    endpoint_roll: float,
    endpoint_pitch: float,
    endpoint_yaw: float,
}

# 수신 클래스
## Cookbot
{
    robot_id: robot_id,
    status: status(recent),
    timestamp: now(),
}
## Pose6D
{
    entity_id: robot_id,
    entity_type: "COOKBOT",
    timestamp: now(),
    x: endpoint_x,
    y: endpoint_y,
    z: endpoint_z,
    roll: endpoint_roll,
    pitch: endpoint_pitch,
    yaw: endpoint_yaw,
}
## JointAngle
{
    robot_id: robot_id,
    timestamp: now(),
    joint_1: angle_1,
    joint_2: angle_2,
    joint_3: angle_3,
    joint_4: angle_4,
    joint_5: angle_5,
    joint_6: angle_6,
}

# 송신 클래스
## 재료 위치 정보
{
    msg_type: "Ingredient",
    ingredient_id: int,
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
}

# 수신 클래스
## Pose6D
{
    entity_id: ingredient_id,
    entity_type: "INVENTORY",
    timestamp: now(),
    x: x,
    y: y,
    z: z,
    roll: roll,
    pitch: pitch,
    yaw: yaw,
}
