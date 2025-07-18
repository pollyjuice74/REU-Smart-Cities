function ptCloud = ros_connection(dataConfig)
    disp("[PENDING] Establishing connection with ROS 'roscore' node");

    % === Connect to ROS master ===
    rosIP = 'http:///192.168.158.156:11311';  % Replace with your containers eth0 IP
    try
        rosinit(rosIP);
        fprintf('[CONNECTED] Connected to ROS container on ip: %s\n', rosIP);
    catch ME
        warning("Could not connect to ROS master: %s", ME.message);
        ptCloud = zeros(100, 3);  % fallback
        return;
    end

    % % === Subscribe to PointCloud2 topic ===
    % topicName = '/velodyne_points';  % or whatever topic your container publishes
    % pause(2);  % give some time for topics to populate
    % try
    %     sub = rossubscriber(topicName, 'sensor_msgs/PointCloud2');
    %     msg = receive(sub, 10);  % wait up to 10 seconds
    %     ptCloud = rosReadXYZ(msg);  % convert to Nx3 matrix
    % catch ME
    %     warning("Could not receive point cloud: %s", ME.message);
    %     ptCloud = zeros(100, 3);  % fallback
    % end
end