function main(varargin)
    clc; 

    % === Handle Command Line Argument ===
    p = inputParser;
    addRequired(p, 'mode', @(x) any(validatestring(x, {'tx', 'rx', 'duplex'})));
    parse(p, varargin{:});
    mode = p.Results.mode;

    % === CONFIG ===
    waveCfg = waveform_config();
    sdrCfg = sdr_config(); 
    dataCfg = data_config();
    viewers = init_viewers();
    dataCfg = random_lidar(dataCfg); % initialize data for tx/rx init

    % === SETTINGS   ===
    sdrCfg.channel = "OverTheAir";
    sdrCfg.displayFlag = false;
    sdrCfg.txID = 'usb:0';
    sdrCfg.rxID = 'usb:0';
    dataCfg.dataSource = "ros";
    dataCfg.subscriberPath = '/local_lidar';
    dataCfg.publisherPath = '/sdr_lidar';

    % --- ROS ---
    if dataCfg.dataSource == "ros"
        dataCfg = ros_connection(dataCfg); % add publisher path
    end

     % --- Initialize TX/RX ---
    [sdrTransmitter, txWaveform, dataCfg, nonHTcfg, sdrCfg, waveCfg] = ...
        init_transmitter(dataCfg, sdrCfg, waveCfg);
    sdrReceiver = init_receiver(txWaveform, sdrCfg, waveCfg);

    % Use onCleanup to ensure resources get released on function exit or error
    cleanupTransmitter = onCleanup(@() safeRelease(sdrTransmitter));
    cleanupReceiver = onCleanup(@() safeRelease(sdrReceiver));

    % --- TX/RX/Duplex Mode ---
    switch mode
        case "tx"
            tx_main(sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg);
        
        case "rx"
            rx_main(txWaveform, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers);
        
        case "duplex"
            % Start simultaneous Tx and Rx
            % Example loop for duplex operation
            disp('Starting duplex Tx/Rx...');

            % Start transmission asynchronously or in background
            tx_main(sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg);

            % Receive in a loop (customize this as needed)
            rx_main(txWaveform, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers); % timeout 0.1 sec
                
            % Stop transmission and release hardware
            release(sdrTransmitter);
            release(sdrReceiver);
            disp('Duplex complete.');
        
        otherwise
            error('Unsupported mode');
    end
end