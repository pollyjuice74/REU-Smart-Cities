function main(varargin)
    clc; 

    % === Handle Command Line Argument ===
    p = inputParser;
    addRequired(p, 'mode', @(x) any(validatestring(x, {'tx', 'rx'})));
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
        otherwise
            error('Unsupported mode');
    end
end
