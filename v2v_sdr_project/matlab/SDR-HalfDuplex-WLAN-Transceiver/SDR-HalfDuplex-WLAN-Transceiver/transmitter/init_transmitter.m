function [sdrTransmitter, txWaveform, dataCfg, nonHTcfg, sdrCfg, waveCfg] = init_transmitter(dataCfg, sdrCfg, waveCfg)

    % Prepare TX Data
    [dataCfg, waveCfg] = prepare_tx_data(dataCfg, waveCfg);  % Or modify prepare_tx_data() to accept bytes
    
    % Create PHY waveform config
    nonHTcfg = phy_config(waveCfg)

    % Generate waveform
    [txWaveform, nonHTcfg, sdrCfg, waveCfg] = generate_waveform(dataCfg, nonHTcfg, sdrCfg, waveCfg);

    if strcmpi(sdrCfg.channel,"OverTheAir")
        sdrTransmitter = sdrtx(sdrCfg.deviceName, ...
            'RadioID', sdrCfg.txID);
        
        sdrTransmitter.BasebandSampleRate = waveCfg.BasebandSampleRate;
        sdrTransmitter.CenterFrequency = sdrCfg.CenterFrequency;
        sdrTransmitter.Gain = sdrCfg.txGain;
        
        if ~strcmpi(sdrCfg.deviceName, "Pluto")
            sdrTransmitter.ShowAdvancedProperties = true;
            sdrTransmitter.BypassUserLogic = true;
        end

    else
        sdrTransmitter = []; % Not needed for simulation
    end