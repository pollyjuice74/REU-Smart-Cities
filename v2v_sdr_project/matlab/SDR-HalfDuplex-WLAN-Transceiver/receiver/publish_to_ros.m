function publish_to_ros(dataCfg)
    % xyz: Nx3 matrix to publish as sensor_msgs/PointCloud2 on /sdr_lidar
    xyz = dataCfg.rxLidarMatrix
    
    if isempty(xyz)
        disp('[ROS] Skipping publish: xyz data is empty.');
        return;
    end

    % Validation
    assert(isnumeric(xyz) && size(xyz,2)>=3, 'xyz must be Nx3 (or more) numeric array');
    assert(all(isfinite(xyz(:))), 'xyz contains non-finite values');

    % ROS Publisher
    pub = rospublisher(dataCfg.publisherPath, 'sensor_msgs/PointCloud2');
    disp(['[ROS] Publisher created on topic: ', dataCfg.publisherPath]);

    % Create message
    msg = rosmessage(pub);
    msg.Header.FrameId = 'map';
    msg.Header.Stamp = rostime('now');

    % Fill message metadata
    msg.Height = 1;
    msg.Width = size(xyz, 1);
    msg.IsBigendian = false;
    msg.IsDense = true;
    msg.PointStep = 12;  % x,y,z floats only (4 bytes * 3)
    msg.RowStep = msg.PointStep * msg.Width;

    % Setup fields x,y,z
    fieldNames = {'x','y','z'};
    offsets = [0,4,8];
    for i = 1:3
        msg.Fields(i) = rosmessage('sensor_msgs/PointField');
        msg.Fields(i).Name = fieldNames{i};
        msg.Fields(i).Offset = offsets(i);
        msg.Fields(i).Datatype = 7; % FLOAT32
        msg.Fields(i).Count = 1;
    end

    % Pack point data
    flat_xyz = reshape(single(xyz(:,1:3))', [], 1);  % ensure Nx3 and single precision
    msg.Data = reshape(typecast(flat_xyz, 'uint8'), 1, []);

    % Final check
    assert(isa(msg.Data,'uint8'), 'msg.Data must be uint8');

    % Send message
    try
        send(pub, msg);
        disp(['[ROS] Published ', num2str(msg.Width), ' points to ', dataCfg.publisherPath]);
    catch ME
        warning('[ROS] Publish failed: %s', ME.message);
        rethrow(ME);
    end
end
