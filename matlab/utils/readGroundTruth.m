function [x, y, theta] = readGroundTruth(gzSub, robotName)
    % readGroundTruth - Reads the robot ground truth pose from Gazebo.
    %
    % This function subscribes to the /gazebo/model_states topic and
    % retrieves the pose of the specified robot model.
    %
    % Inputs:
    %   gzSub     - ROS subscriber for /gazebo/model_states
    %   robotName - Name of the robot model in Gazebo
    %
    % Outputs:
    %   x     - Robot x position [m]
    %   y     - Robot y position [m]
    %   theta - Robot yaw angle [rad]

    % Receive Gazebo model states message
    gzMsg = receive(gzSub, 3);

    % Find robot index in model list
    idx = find(strcmp(gzMsg.Name, robotName), 1);

    if isempty(idx)
        error('Robot model "%s" not found in Gazebo.', robotName);
    end

    % Extract robot position
    x = gzMsg.Pose(idx).Position.X;
    y = gzMsg.Pose(idx).Position.Y;

    % Extract robot orientation quaternion
    quat = gzMsg.Pose(idx).Orientation;

    % Convert quaternion to Euler angles
    eul = quat2eul([quat.W quat.X quat.Y quat.Z]);

    % Extract yaw angle
    theta = normalizeAngle(eul(1));
end