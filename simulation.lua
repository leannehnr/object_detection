sim = require('sim')
simROS2 = require('simROS2')

--------------------------------------------------
-- Callback joints
--------------------------------------------------
function joint0Callback(msg)
    sim.setJointTargetPosition(armJoints[1], msg.data)
end

function joint1Callback(msg)
    sim.setJointTargetPosition(armJoints[2], msg.data)
end

function joint2Callback(msg)
    sim.setJointTargetPosition(armJoints[3], msg.data)
end

function joint3Callback(msg)
    sim.setJointTargetPosition(armJoints[4], msg.data)
end

function joint4Callback(msg)
    sim.setJointTargetPosition(armJoints[5], msg.data)
end

function gripper1Callback(msg)
    sim.setJointTargetPosition(gripperJoints[1], msg.data)
end

function gripper2Callback(msg)
    sim.setJointTargetPosition(gripperJoints[2], msg.data)
end

--------------------------------------------------
-- Callback cmd_vel (base)
-- Convention voulue:
-- +vx : avance vers le bras
-- +vy : va vers la gauche (vue depuis le bras)
-- +wz : rotation anti-horaire
--------------------------------------------------
function cmdVelCallback(msg)
    -- Inversions (validées chez toi)
    cmd_vx = -msg.linear.x
    cmd_vy = -msg.linear.y
    cmd_wz =  msg.angular.z
end

--------------------------------------------------
function sysCall_init()
    -- handle de base robot (script attaché au robot)
    robotHandle = sim.getObject('.')

    armJoints = {}
    for i = 0, 4 do
        armJoints[i + 1] = sim.getObject('../youBotArmJoint' .. i)
    end

    -- pince (2 joints séparés)
    gripperJoints = {
        sim.getObject('../youBotGripperJoint1'),
        sim.getObject('../youBotGripperJoint2')
    }

    -- roues
    wheelFL = sim.getObject('../Joint/Rectangle0/rollingJoint_fl')
    wheelFR = sim.getObject('../Joint/Rectangle0/rollingJoint_fr')
    wheelRL = sim.getObject('../rollingJoint_rl')
    wheelRR = sim.getObject('../rollingJoint_rr')

    sim.setJointTargetForce(wheelFL, 50)
    sim.setJointTargetForce(wheelFR, 50)
    sim.setJointTargetForce(wheelRL, 50)
    sim.setJointTargetForce(wheelRR, 50)

    -- paramètres base (à ajuster si besoin)
    r  = 0.0475     -- rayon roue (m) approx
    lx = 0.235/2    -- demi-longueur (m) approx
    ly = 0.150/2    -- demi-largeur (m) approx
    k  = lx + ly

    -- commandes courantes
    cmd_vx, cmd_vy, cmd_wz = 0.0, 0.0, 0.0

    -- Subscriptions ROS2 arm
    sub0 = simROS2.createSubscription('/youbot/arm/joint0_cmd','std_msgs/msg/Float64','joint0Callback')
    sub1 = simROS2.createSubscription('/youbot/arm/joint1_cmd','std_msgs/msg/Float64','joint1Callback')
    sub2 = simROS2.createSubscription('/youbot/arm/joint2_cmd','std_msgs/msg/Float64','joint2Callback')
    sub3 = simROS2.createSubscription('/youbot/arm/joint3_cmd','std_msgs/msg/Float64','joint3Callback')
    sub4 = simROS2.createSubscription('/youbot/arm/joint4_cmd','std_msgs/msg/Float64','joint4Callback')

    sub5 = simROS2.createSubscription('/youbot/arm/gripper1_cmd','std_msgs/msg/Float64','gripper1Callback')
    sub6 = simROS2.createSubscription('/youbot/arm/gripper2_cmd','std_msgs/msg/Float64','gripper2Callback')

    -- Subscription ROS2 base
    subCmdVel = simROS2.createSubscription('/cmd_vel', 'geometry_msgs/msg/Twist', 'cmdVelCallback')

    -- Publisher odom
    pubOdom = simROS2.createPublisher('/odom', 'nav_msgs/msg/Odometry')
end

--------------------------------------------------
-- Applique cmd_vel à chaque step simu
--------------------------------------------------
function sysCall_actuation()
    -- conversion mecanum: vx, vy, wz -> vitesses roues (rad/s)
    local wFL = (cmd_vx - cmd_vy - k*cmd_wz) / r
    local wFR = (cmd_vx + cmd_vy + k*cmd_wz) / r
    local wRL = (cmd_vx + cmd_vy - k*cmd_wz) / r
    local wRR = (cmd_vx - cmd_vy + k*cmd_wz) / r

    sim.setJointTargetVelocity(wheelFL, wFL)
    sim.setJointTargetVelocity(wheelFR, wFR)
    sim.setJointTargetVelocity(wheelRL, wRL)
    sim.setJointTargetVelocity(wheelRR, wRR)
end

--------------------------------------------------
-- Publie /odom (position + orientation quaternion)
--------------------------------------------------
function sysCall_sensing()
    local pos = sim.getObjectPosition(robotHandle, -1)      -- world
    local quat = sim.getObjectQuaternion(robotHandle, -1)   -- world

    local odom = {}

    -- header
    odom.header = {}
    odom.header.frame_id = 'odom'

    -- Timestamp: selon versions de simROS2, getTime() est dispo.
    -- Si tu as une erreur ici, supprime juste la ligne suivante.
    odom.header.stamp = simROS2.getTime()

    odom.child_frame_id = 'base_link'

    -- pose
    odom.pose = { pose = {} }
    odom.pose.pose.position = { x = pos[1], y = pos[2], z = pos[3] }
    odom.pose.pose.orientation = { x = quat[1], y = quat[2], z = quat[3], w = quat[4] }

    -- twist (optionnel mais utile) : on met la commande courante
    odom.twist = { twist = {} }
    odom.twist.twist.linear  = { x = cmd_vx, y = cmd_vy, z = 0.0 }
    odom.twist.twist.angular = { x = 0.0, y = 0.0, z = cmd_wz }

    simROS2.publish(pubOdom, odom)
end
