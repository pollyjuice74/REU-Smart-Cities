function rxWaveform = post_rx_filtering(rxWaveform, waveCfg, nonHTcfg)
    aStop = 40; % Stopband attenuation
    ofdmInfo = wlanNonHTOFDMInfo('NonHT-Data',nonHTcfg); % OFDM parameters
    SCS = waveCfg.sampleRate/ofdmInfo.FFTLength; % Subcarrier spacing
    txbw = max(abs(ofdmInfo.ActiveFrequencyIndices))*2*SCS; % Occupied bandwidth
    
    [L,M] = rat(1/waveCfg.osf);
    maxLM = max([L M]);
    R = (waveCfg.sampleRate-txbw)/waveCfg.sampleRate;
    TW = 2*R/maxLM; % Transition width

    b = designMultirateFIR(L,M,TW,aStop);
    firrc = dsp.FIRRateConverter(L,M,b); %  changes the sampling rate of a signal, either increasing it (upsampling) or decreasing it (downsampling)
    rxWaveform = firrc(rxWaveform);
end
