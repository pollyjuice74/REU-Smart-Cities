function publish_pointcloud(ptCloud, frame_id)
    % ptCloud: Nx3 matrix of XYZ points
    % frame_id: TF frame (e.g., 'base_link' or 'sdr_frame')

    msg = rosmessage('sensor_msgs/PointCloud2');
    msg.Header.Stamp = rostime('now');
    msg.Header.FrameId = frame_id;

    % Fill in fields for XYZ point cloud
    ptCloudMsg = rosWriteXYZ(msg, ptCloud);

    pub = rospublisher('/sdr_points', 'sensor_msgs/PointCloud2');
    pause(0.5); % allow publisher setup

    send(pub, ptCloudMsg);
    disp('[ROS] Published SDR point cloud to /sdr_points');
end
