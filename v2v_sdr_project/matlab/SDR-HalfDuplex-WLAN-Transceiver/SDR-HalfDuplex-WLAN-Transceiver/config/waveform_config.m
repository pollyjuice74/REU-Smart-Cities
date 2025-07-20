function waveCfg = waveform_config()
    waveCfg.msduLength = 2304;
    waveCfg.osf = 1.5;
    waveCfg.MCS = 4; % 16QAM 3/4
    waveCfg.BW = 'CBW10';

    % Set dynamically
    waveCfg.BasebandSampleRate = [];
    waveCfg.sampleRate = [];
    waveCfg.numMSDUs = [];
    waveCfg.lengthMPDU = [];
end
