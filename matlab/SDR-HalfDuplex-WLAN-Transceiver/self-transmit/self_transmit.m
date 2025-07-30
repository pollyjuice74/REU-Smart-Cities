function self_transmit(txWaveform, sdrTransmitter, sdrReceiver, ...
                dataCfg, nonHTcfg, sdrCfg, waveCfg, ...
                viewers)
    % Start simultaneous Tx and Rx
    % Example loop for duplex operation
    disp('Starting duplex Tx/Rx...');
    transmitRepeat_flag = true

    while true    
        % Start transmission asynchronously or in background
        tx_main(sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg, transmitRepeat_flag);
    
        % Receive in a loop (customize this as needed)
        [dataCfg, sdrReceiver] = rx_main(txWaveform, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers); % timeout 0.1 sec
    end

    % % Stop transmission and release hardware
    % release(sdrTransmitter);
    % release(sdrReceiver);
    % disp('Duplex complete.');
end