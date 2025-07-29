function [txWaveform, nonHTcfg, sdrCfg, waveCfg] = generate_waveform(dataCfg, nonHTcfg, sdrCfg, waveCfg)
    % Variables
    msduLength = waveCfg.msduLength;
    osf = waveCfg.osf;
    txDataBits = dataCfg.txDataBits;

    % --- Fragment and Encode into MPDUs ---
    numMSDUs = ceil(length(txDataBits)/8 / msduLength);
    padZeros = msduLength*8 - mod(length(txDataBits), msduLength*8);
    txDataBits = [txDataBits; zeros(padZeros,1)];
    
    data = zeros(0,1); % Accumulator for PSDUs
    for i = 0:numMSDUs-1
        % Get MSDU payload (in bits)
        bitsStart = i*msduLength*8 + 1;
        bitsEnd = (i+1)*msduLength*8;
        frameBody = bit2int(txDataBits(bitsStart:bitsEnd), 8, false);

        % MAC Frame
        cfgMAC = wlanMACFrameConfig('FrameType','Data','SequenceNumber',i);
        [psdu, lengthMPDU] = wlanMACFrame(frameBody, cfgMAC, 'OutputFormat', 'bits');
        data = [data; psdu]; %#ok<AGROW>
    end

    scramblerInitialization = randi([1 127], numMSDUs, 1);
    waveCfg.sampleRate = wlanSampleRate(nonHTcfg);
    waveCfg.BasebandSampleRate = waveCfg.sampleRate * waveCfg.osf

    % --- Generate waveform ---
    txWaveform = wlanWaveformGenerator(data, nonHTcfg, ...
        'NumPackets', numMSDUs, ...
        'IdleTime', 20e-6, ...
        'ScramblerInitialization', scramblerInitialization, ...
        'OversamplingFactor', osf);
        
    sdrCfg.SamplesPerFrame = length(txWaveform) * 2;
end
