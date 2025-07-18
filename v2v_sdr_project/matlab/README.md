# V2V Fusion

```
v2v_fusion_project/
│
├── main.m                         % Entry point (initializes everything)
├── config.m                      % Simulation constants, topic names, IPs
│
├── lidar/
│   ├── subscribeLidar.m         % Subscribe and receive point cloud
│   ├── fusePointClouds.m        % Merge incoming + local point clouds
│   └── transformPointCloud.m    % Optional ICP alignment or filtering
│
├── communication/
│   ├── sendLidarOverDSRC.m      % Pluto: send LiDAR over DSRC
│   ├── receiveLidarOverDSRC.m   % Pluto: receive and decode LiDAR
│   └── toggleMode.m             % CAV/RSU logic and switching
│
├── visualization/
│   ├── showPointCloud.m         % Show real-time point cloud
│   └── logMapState.m            % Save fused maps to disk or file

```