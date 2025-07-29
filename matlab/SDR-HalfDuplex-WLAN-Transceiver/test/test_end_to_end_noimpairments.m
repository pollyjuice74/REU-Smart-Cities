function test_end_to_end_noimpairments()
% End-to-end test with ideal channel (no noise)

    clc; close all;
    fprintf('[TEST] Starting end-to-end transmission with no impairments...\n');

    % === CONFIG ===
    sdrCfg = sdr_config(); 
    sdrCfg.channel = "NoImpairments"; % Force override for test
    waveCfg = waveform_config();
    dataCfg = data_config(); % not really using this cfg, should make a get_lidar_data(dataCFG) function in data/
    
    % === Viewers ===
    viewers = init_viewers(); % spectrumScope, constellation plotting

    % === Generate Dummy LiDAR ===
    [dataCfg] = random_lidar(dataCfg);

    % === TRANSMIT ===
    fprintf('[TEST] Transmitting...\n');
    [txWaveform, sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg] = tx_main(dataCfg, sdrCfg, waveCfg);

    % === RECEIVE ===
    fprintf('[TEST] Receiving...\n');
    [dataCfg] = rx_main(txWaveform, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers);

    % === COMPARE ===
    N = min(length(dataCfg.txDataBits), length(dataCfg.rxDataBits));
    txBits = dataCfg.txDataBits(1:N);
    rxBits = dataCfg.rxDataBits(1:N);

    if isequal(txBits, rxBits)
        fprintf('[PASS] Bitstream matched! %d bits transmitted and received.\n', N);
    else
        diff = sum(txBits ~= rxBits);
        fprintf('[FAIL] %d bit errors out of %d bits.\n', diff, N);
    end
end
