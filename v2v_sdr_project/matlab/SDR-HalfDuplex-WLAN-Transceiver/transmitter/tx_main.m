% tx_main.m – Transmitter entry point
function [txWaveform, sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg] = tx_main(sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg)
    if dataCfg.dataSource == "ros" 
        dataCfg = ros_subscribe_local(dataCfg) % create this one
    end

    % Prepare TX Data
    [dataCfg, waveCfg] = prepare_tx_data(dataCfg, waveCfg);  % Or modify prepare_tx_data() to accept bytes
    
    % Create PHY waveform config
    nonHTcfg = phy_config(waveCfg)

    % Generate waveform
    [txWaveform, nonHTcfg, sdrCfg, waveCfg] = generate_waveform(dataCfg, nonHTcfg, sdrCfg, waveCfg);

    % Transmit (optional)
    transmit_rf(sdrTransmitter, txWaveform);
end

    % 
    % % === Prepare publisher ===
    % sdrPub = rospublisher(dataCfg.publisherPath, 'sensor_msgs/PointCloud2');
    % fprintf('[CONNECTED] Connected to ROS sdr publisher on %s', dataCfg.publisherPath);
    % msg = rosmessage(sdrPub);
    % 
    % % === Send dummy or real data ===
    % try
    %     % Replace with your real RX logic
    %     for i = 1:100  % Just send 5 packets for now
    %          % === Generate random lidar data
    %         dataCfg = random_lidar(dataCfg);  % This sets txLidarMatrix
    %         xyz = dataCfg.txLidarMatrix;
    % 
    %         % === Fill in required fields
    %         msg = rosmessage(sdrPub);
    % 
    %         msg.Header.FrameId = 'map';
    %         msg.Height = 1;
    %         msg.Width = size(xyz, 1);
    %         msg.IsDense = true;
    %         msg.IsBigendian = false;
    %         msg.PointStep = 12;  % 3 fields * 4 bytes each
    %         msg.RowStep = msg.PointStep * msg.Width;
    % 
    %         % Define Fields: x, y, z
    %         msg.Fields(1).Name = 'x';
    %         msg.Fields(1).Offset = 0;
    %         msg.Fields(1).Datatype = ros.msg.sensor_msgs.PointField.FLOAT32;
    %         msg.Fields(1).Count = 1;
    % 
    %         msg.Fields(2).Name = 'y';
    %         msg.Fields(2).Offset = 4;
    %         msg.Fields(2).Datatype = ros.msg.sensor_msgs.PointField.FLOAT32;
    %         msg.Fields(2).Count = 1;
    % 
    %         msg.Fields(3).Name = 'z';
    %         msg.Fields(3).Offset = 8;
    %         msg.Fields(3).Datatype = ros.msg.sensor_msgs.PointField.FLOAT32;
    %         msg.Fields(3).Count = 1;
    % 
    %         % === Encode XYZ into byte stream
    %         flat_xyz = xyz';  % transpose to 3 x N
    %         msg.Data = typecast(single(flat_xyz(:)), 'uint8');
    % 
    %         % === Publish to /sdr_lidar
    %         send(sdrPub, msg);
    %         fprintf('[INFO] Published valid PointCloud2 to /sdr_lidar with %d points\n', msg.Width);
    %         pause(0.1);
    %     end
    % 
    % catch ME
    %     warning("Could not publish SDR data: %s", ME.message);
    % end
    % 
    % % Return something (dummy for now)
    % ptCloud = rand(100, 3);
