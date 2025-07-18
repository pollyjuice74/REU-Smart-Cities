% rx_main.m – Receiver entry point
function [dataCfg, sdrReceiver] = rx_main(txWaveform, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers)
    maxRetries = 10; % number of receive attempts in warmup
    retryCount = 0;
    rxWaveform = [];

    while true
        % === Receive Waveform ===
        try
            [rxWaveform] = receive_rf(txWaveform, sdrReceiver, sdrCfg, waveCfg);  % case logic for simulated/real transmission

            % === Process Received Signal ===
            [dataCfg] = process_received_signal(rxWaveform, dataCfg, waveCfg, sdrCfg, nonHTcfg, viewers);

        catch ME
            fprintf('[WARN] Receive attempt %d failed: %s\n', retryCount+1, ME.message);
            
            pause(0.5); % Optional: pause to allow transmitter/warmup to stabilize
            retryCount = retryCount + 1;
        end
    end

    if ~isempty(rxWaveform)
        warning('Reception failed after %d attempts.', maxRetries);
        % === Reconstruct Data ===
        reconstruct_data(waveCfg, dataCfg); if sdrCfg.displayFlag==true end
    end

end