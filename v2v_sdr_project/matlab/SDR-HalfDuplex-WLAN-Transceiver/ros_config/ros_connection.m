function dataCfg = ros_connection(dataCfg)
    disp("[PENDING] Establishing connection with ROS 'roscore' node");

    % === Variables === % put in dataCfg
    rosIP = '192.168.158.156';  % <- FIX: Remove 'http://'
    rosPort = 11311;

    % === Connect to ROS master ===
    try
        rosshutdown
        rosinit(rosIP, rosPort);  % <- Use IP and Port separately
        fprintf('[CONNECTED] Connected to ROS container on ip: %s\n', rosIP);
    catch ME
        warning("Could not connect to ROS master: %s", ME.message);
        ptCloud = zeros(100, 3);  % fallback
        return;
    end
end