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

    % === Prepare publisher ===
    sdrPub = rospublisher(dataCfg.publisherPath, 'std_msgs/Float64MultiArray');
    fprintf('[CONNECTED] Connected to ROS sdr publisher on %s', dataCfg.publisherPath);
    msg = rosmessage(sdrPub);

    % === Send dummy or real data ===
    try
        % Replace with your real RX logic
        for i = 1:5  % Just send 5 packets for now
            dummyRx = rand(1, 64);  % Replace with: rx_main() or receive(...)
            msg.Data = dummyRx;
            send(sdrPub, msg);
            disp("[INFO] Sent SDR packet to ROS.");
            pause(0.5);
        end

    catch ME
        warning("Could not publish SDR data: %s", ME.message);
    end

    % Return something (dummy for now)
    ptCloud = rand(100, 3);
end