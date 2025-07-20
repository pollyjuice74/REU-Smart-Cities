% ===== ROS LiDAR PointCloud data =====
rosinit('http://192.168.158.156:11311') % Should dynamically get ip address

lidarSub = rossubscriber('/velodyne_points', 'sensor_msgs/PointCloud2');

% Get latest point cloud
rosPC = receive(lidarSub);
ptCloud = rosReadXYZ(rosPC); % Nx3 matrix

lidarInt = int16(ptCloud * 100);             % Quantize
txLidarBytes = typecast(lidarInt(:), 'uint8'); % Flatten