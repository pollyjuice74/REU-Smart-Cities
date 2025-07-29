function discovery_main(txWaveform, sdrTransmitter, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers)
    disp('Starting ALOHA-based discovery mode...');

    alohaProbability = 1.0;                    % TX less often to RX more often
    alohaBackoff = @() 0.3 + rand() * 0.4;    % shorter backoff after TX
    rxDuration = 1.0;                         % listen longer per RX phase
    maxIterations = 1000;

    for iter = 1:maxIterations
        % Remove or minimize this pause to speed iterations
        % pause(0.01); 

        if rand() < alohaProbability
            disp('[TX] Transmitting ALOHA packet...');
            fprintf('[TX] Sent at %s\n', datestr(now, 'HH:MM:SS'));
            tx_main(sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg);

            pause(alohaBackoff());
        else
            disp('[RX] Receiving ALOHA packet...');
            tStart = tic;
            while toc(tStart) < rxDuration
                [dataCfg, ~] = rx_main(txWaveform, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers);
            end
        end
    end

    release(sdrTransmitter);
    release(sdrReceiver);
    disp('ALOHA Discovery complete.');
end
