function discovery_main(txWaveform, sdrTransmitter, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers)
    disp('Starting ALOHA-based discovery mode...');

    % --- Constants ---
    alohaProbability = 0.4;          % Chance to transmit in each iteration
    alohaBackoff = @() 0.5 + rand() * 0.5;     % Backoff time (0 to 5 seconds)
    maxIterations = 1000

    for iter = 1:maxIterations
        pause(0.02);

        if rand() < alohaProbability
            % --- TRANSMIT ---
            disp('[TX] Transmitting ALOHA packet...');
            fprintf('[TX] Sent at %s\n', datestr(now, 'HH:MM:SS'));
            tx_main(sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg);  % Async transmit
            pause(alohaBackoff());  % Random backoff
        else
            % --- RECEIVE ---
            disp('[RX] Receiveing ALOHA packet...');
            [dataCfg, ~] = rx_main(txWaveform, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers);  % Blocking receive    
        end
    end

    % --- Cleanup ---
    release(sdrTransmitter);
    release(sdrReceiver);
    disp('ALOHA Discovery complete.');
end