function [dataCfg] = random_lidar(dataCfg)
    % Generate dummy point cloud data
    ptCloud = pointCloud(rand(1000,3)); % [x y z] points
    xyz = ptCloud.Location;

    % Optional: add fake intensity
    intensity = []; % or: rand(1000,1);

    if isempty(intensity)
        txLidarMatrix = xyz; % Nx3
    else
        txLidarMatrix = [xyz, intensity]; % Nx4
    end

    % Scale and quantize to send as bytes
    lidarInt = int16(txLidarMatrix * dataCfg.scaleFactor);
    txLidarBytes = typecast(lidarInt(:), 'uint8');

    % Set variables
    dataCfg.txLidarBytes = txLidarBytes
    dataCfg.txLidarMatrix = txLidarMatrix

end
