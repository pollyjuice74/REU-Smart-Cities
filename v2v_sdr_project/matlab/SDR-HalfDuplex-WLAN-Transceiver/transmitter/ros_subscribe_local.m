function dataCfg = ros_subscribe_local(dataCfg)

    % Subscribe to the topic and assign the callback
    localSub = rossubscriber(dataCfg.subscriberPath, @(~, msg) lidarCallback(msg));

    disp(['[ROS] Subscribed to topic: ', dataCfg.subscriberPath]);

    % Store subscriber object in dataCfg for future reference if needed
    dataCfg.localSub = localSub;
end

function lidarCallback(msg)
    % Display reception
    disp('[ROS] PointCloud2 message received (callback).');

    % Decode the PointCloud2 message into xyz points
    xyz = decodePointCloud2(msg);

    % Display result (you could also store globally or trigger visualization)
    disp(['[ROS] Received ', num2str(size(xyz,1)), ' points.']);

    % Optional: visualize
    pcshow(xyz);

    % Optional: you can store this in a persistent or global if needed for downstream use
    % persistent latestPointCloud;
    % latestPointCloud = xyz;
end

function xyz = decodePointCloud2(msg)
    % Decode sensor_msgs/PointCloud2 to Nx3 float array (x,y,z)
    data = msg.Data;
    width = msg.Width;
    height = msg.Height;
    pointStep = msg.PointStep;

    numPoints = width * height;
    xyz = zeros(numPoints, 3);

    for i = 1:numPoints
        offset = (i-1)*pointStep + 1;
        x_bytes = data(offset:offset+3);
        y_bytes = data(offset+4:offset+7);
        z_bytes = data(offset+8:offset+11);

        xyz(i,1) = typecast(uint8(x_bytes), 'single');
        xyz(i,2) = typecast(uint8(y_bytes), 'single');
        xyz(i,3) = typecast(uint8(z_bytes), 'single');
    end
end
