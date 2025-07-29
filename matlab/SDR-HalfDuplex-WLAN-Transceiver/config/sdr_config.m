function sdrCfg = sdr_config()
    sdrCfg.channel = "OverTheAir"; % Options: "OverTheAir", "NoImpairments", "GaussianNoise"
    sdrCfg.deviceName = "Pluto"; % Options: "Pluto", "E3xx"
    sdrCfg.channelNumber = 5;
    sdrCfg.frequencyBand = 2.4; % GHz Options: "2.4", "5.9"
    sdrCfg.CenterFrequency = wlanChannelFrequency(sdrCfg.channelNumber, sdrCfg.frequencyBand);
    sdrCfg.txGain = 0;
    sdrCfg.rxGain = 30; % If not using AGC
    sdrCfg.displayFlag = true; 

    % Set rx/tx device IDs based on connected Plutos
    devices = findPlutoRadio;
    if numel(devices) >= 2
        [sdrCfg.txID, sdrCfg.rxID] = deal(devices(1).RadioID, devices(2).RadioID);
    elseif numel(devices) == 1
        [sdrCfg.txID, sdrCfg.rxID] = deal(devices(1).RadioID);
    else
        % Default fallback (likely works if one Pluto is connected)
        warning("No Pluto devices found with `findPlutoRadio`. Falling back to 'usb:0'.");
        [sdrCfg.txID, sdrCfg.rxID] = deal("usb:0");
    end

    % rx
    sdrCfg.GainSource = 'Manual'; %'AGC Fast Attack';
    sdrCfg.OutputDataType =  'double';
end
