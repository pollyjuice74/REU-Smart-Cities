function test_end_to_end_ota()
% End-to-end over-the-air test using real SDR hardware (e.g., PlutoSDR)

    clc; close all;
    fprintf('[TEST] Starting over-the-air transmission test...\n');

    % === CONFIG ===
    sdrCfg = sdr_config(); 
    sdrCfg.channel = "OverTheAir"; % mark this as OTA test
    waveCfg = waveform_config();
    dataCfg = data_config();
    
    % === Viewers ===
    viewers = init_viewers(); % Optional: spectrum/constellation

    % === Generate Dummy LiDAR ===
    dataCfg = random_lidar(dataCfg);

    fprintf('Warming up hardware...\n');
    for k = 1:5
        [txWaveform, ~, dataCfg, ~, ~, ~] = tx_main(dataCfg, sdrCfg, waveCfg);
        [dataCfg, sdrReceiver] = rx_main(txWaveform, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers);
        release(sdrReceiver);
        pause(0.2);
    end
    fprintf('Warm-up complete.\n');

    % === Transmit ===
    fprintf('[TEST] Transmitting...\n');
    [txWaveform, sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg] = tx_main(dataCfg, sdrCfg, waveCfg);

    % === Receive ===
    fprintf('[TEST] Receiving...\n');
    [dataCfg, sdrReceiver] = rx_main(txWaveform, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers);
    release(sdrTransmitter); % should be if statement if not testing

    % === Compare ===
    N = min(length(dataCfg.txDataBits), length(dataCfg.rxDataBits));
    txBits = dataCfg.txDataBits(1:N);
    rxBits = dataCfg.rxDataBits(1:N);
        
    % Visualize errors
    [ber, bitErrors] = plot_bit_errors(txBits, rxBits, 100);  % Adjust row count as needed

    % === Result Logging ===
    tolerance = 1e-3;  % Tolerate BER up to 0.1%
    if ber <= tolerance
        fprintf('[PASS] %d bits checked. %d bit errors (BER = %.2e).\n', N, bitErrors, ber);
    else
        fprintf('[WARNING] %d bits checked. %d bit errors (BER = %.2e).\n', N, bitErrors, ber);
        idx = find(txBits ~= rxBits);
        if ~isempty(idx)
            fprintf('Example mismatch at bit %d: tx=%d, rx=%d\n', idx(1), txBits(idx(1)), rxBits(idx(1)));
        end
    end
    release(sdrReceiver); % clean up SDR device
end
