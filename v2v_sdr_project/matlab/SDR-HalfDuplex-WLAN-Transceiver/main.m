function main(varargin)
    clc; 

    % === Handle Command Line Argument ===
    p = inputParser;
    addRequired(p, 'mode', @(x) any(validatestring(x, {'tx', 'rx', 'half-duplex', 'full-duplex'})));
    parse(p, varargin{:});
    mode = p.Results.mode;

    % === CONFIG ===
    waveCfg = waveform_config();
    sdrCfg = sdr_config(); 
    dataCfg = data_config();
    viewers = init_viewers();
    dataCfg = random_lidar(dataCfg);

    % === SETTINGS   ===
    sdrCfg.channel = "OverTheAir";
    dataCfg.dataSource = "ros";

    % --- ROS ---
    if dataCfg.dataSource == "ros"
        ptCloud = ros_connection(dataCfg);
    else
        dataCfg = random_lidar(dataCfg);
    end

    % --- TX/RX Mode ---
    switch mode
        case "tx"
            sdrCfg.txID = 'usb:0';
            [sdrTransmitter, ~, dataCfg, nonHTcfg, sdrCfg, waveCfg] = ...
                init_transmitter(dataCfg, sdrCfg, waveCfg);
            tx_main(sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg);
        
        case "rx"
            sdrCfg.rxID = 'usb:0';
            [~, txWaveform, dataCfg, nonHTcfg, sdrCfg, waveCfg] = ...
                init_transmitter(dataCfg, sdrCfg, waveCfg);
            sdrReceiver = init_receiver(txWaveform, sdrCfg, waveCfg);
            rx_main(txWaveform, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers);
        
        case "half-duplex"
            % Use the same device for Tx and Rx
            sdrCfg.txID = 'usb:0';
            sdrCfg.rxID = 'usb:0';

            % Initialize transmitter and receiver
            [sdrTransmitter, txWaveform, dataCfg, nonHTcfg, sdrCfg, waveCfg] = ...
                init_transmitter(dataCfg, sdrCfg, waveCfg);
            sdrReceiver = init_receiver(txWaveform, sdrCfg, waveCfg);

            % Start simultaneous Tx and Rx
            % Example loop for duplex operation
            disp('Starting half-duplex Tx/Rx...');

            % Start transmission asynchronously or in background
            tx_main(sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg);

            % Receive in a loop (customize this as needed)
            for idx = 1:100
                rxData = rx_main(txWaveform, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers); % timeout 0.1 sec
                % Process received data, e.g., decode LiDAR points or store
                % You can compare/process with ROS LiDAR here
                disp(['Received frame ', num2str(idx)]);
                
                % Optionally visualize or save rxData here
            end

            % Stop transmission and release hardware
            release(sdrTransmitter);
            release(sdrReceiver);
            disp('Half-duplex Tx/Rx complete.');
        
        case "full-duplex"
            % % Assign separate devices
            % sdrCfg.txID = 'usb:0';  % Transmit Pluto
            % sdrCfg.rxID = 'usb:1';  % Receive Pluto
            % 
            % % Init Tx and Rx
            % [sdrTransmitter, txWaveform, dataCfg, nonHTcfg, sdrCfg, waveCfg] = ...
            %     init_transmitter(dataCfg, sdrCfg, waveCfg);
            % sdrReceiver = init_receiver(txWaveform, sdrCfg, waveCfg);
            % 
            % % Start transmission using transmitRepeat for continuous signal
            % transmitRepeat(sdrTransmitter, txWaveform);  % works like async
            % 
            % disp('Starting full-duplex Tx/Rx...');
            % 
            % % Simultaneous Rx loop
            % for idx = 1:100
            %     rxData = receive(sdrReceiver, 0.1);
            %     % Process rxData
            %     disp(['Received frame ', num2str(idx)]);
            % end
            % 
            % % Cleanup
            % release(sdrTransmitter);
            % release(sdrReceiver);
            % disp('Full-duplex Tx/Rx complete.');

        otherwise
            error('Unsupported mode');
    end
end
