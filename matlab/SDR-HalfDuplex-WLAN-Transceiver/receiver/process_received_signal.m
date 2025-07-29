function [dataCfg] = process_received_signal(rxWaveform, dataCfg, waveCfg, sdrCfg, nonHTcfg, viewers)
    % === Filter Received Waveform ===
    rxWaveform = post_rx_filtering(rxWaveform, waveCfg, nonHTcfg);
    
    % === Decode PSDU MAC from Waveform ===
    fprintf('[RX] Decoding waveform...\n');
    [dataCfg] = psdu_mac_decode(rxWaveform, dataCfg, waveCfg, nonHTcfg, sdrCfg, viewers);
    
   