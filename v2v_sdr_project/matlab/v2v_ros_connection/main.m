rosinit;

% Load config
cfg = config();

% Init subscribers
lidarSub = rossubscriber(cfg.lidarTopic, 'sensor_msgs/PointCloud2');

% Loop: receive → fuse → transmit/receive → visualize
while true
    ptCloudLocal = subscribeLidar(lidarSub);

    if cfg.CAV_mode
        fusedMap = fusePointClouds(ptCloudLocal, cfg.globalMap);
    else
        ptCloudRemote = receiveLidarOverDSRC();   % from Pluto
        fusedMap = fusePointClouds(ptCloudLocal, ptCloudRemote);
    end

    cfg.globalMap = fusedMap;

    % Send your local data out
    sendLidarOverDSRC(ptCloudLocal);

    % Show
    showPointCloud(cfg.globalMap);

    pause(0.1);  % simulate frame rate
end
