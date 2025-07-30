function main(varargin)
    clc; 

    % === Handle Command Line Argument ===
    p = inputParser;
    addRequired(p, 'mode', @(x) any(validatestring(x, {'tx', 'rx', 'self-transmit', 'discovery'})));
    parse(p, varargin{:});
    mode = p.Results.mode;

    % === CONFIG ===
    waveCfg = waveform_config();
    sdrCfg = sdr_config(); 
    dataCfg = data_config();
    dataCfg = random_lidar(dataCfg); % initialize data for tx/rx init

    % === PLOTTING ===
    viewers = init_viewers();
    viewers.logStats = make_metrics_logger(viewers);

    % === SETTINGS   ===
    sdrCfg.channel = "OverTheAir";
    sdrCfg.displayFlag = true;
    sdrCfg.txID = 'usb:0';
    sdrCfg.rxID = 'usb:0';
    sdrCfg.txGain = -10;
    dataCfg.dataSource = "random";
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

    disp(waveCfg)

    % % Lists for time series display/statistical analysis
    % global evmHistory berHistory;
    % evmHistory = [];
    % berHistory = [];
    % 
    % global packetStats;
    % if isempty(packetStats)
    %     packetStats = struct( ...
    %         'timestamp', {}, ...
    %         'ber', {}, ...
    %         'evm', {}, ...
    %         'snr', {}, ...
    %         'packetSeq', {}, ...
    %         'valid', {}, ...
    %         'length', {});
    % end
    % 

    % --- TX/RX/Duplex Mode ---
    switch mode
        case "tx"
            transmitRepeat_flag = true
            while true
                tx_main(sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg, transmitRepeat_flag);
            end

        case "rx"
            while true
                rx_main(txWaveform, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers);
            end
                
        case "self-transmit"
            self_transmit(txWaveform, sdrTransmitter, sdrReceiver, ...
                dataCfg, nonHTcfg, sdrCfg, waveCfg, ...
                viewers);

        case "discovery"
            discovery_main(txWaveform, sdrTransmitter, sdrReceiver, ...
                dataCfg, nonHTcfg, sdrCfg, waveCfg, ...
                viewers);
        
        otherwise
            error('Unsupported mode');
    end
end