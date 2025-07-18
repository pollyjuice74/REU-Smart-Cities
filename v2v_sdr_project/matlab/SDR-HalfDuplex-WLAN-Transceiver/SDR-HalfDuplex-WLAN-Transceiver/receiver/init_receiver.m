function sdrReceiver = init_receiver(txWaveform, sdrCfg, waveCfg)

    switch sdrCfg.channel
        case 'OverTheAir'
            sdrReceiver = sdrrx(sdrCfg.deviceName, ...
                'RadioID', sdrCfg.rxID);
            
            sdrReceiver.BasebandSampleRate = waveCfg.BasebandSampleRate;
            sdrReceiver.CenterFrequency = sdrCfg.CenterFrequency;
            sdrReceiver.OutputDataType = sdrCfg.OutputDataType;
            sdrReceiver.GainSource = sdrCfg.GainSource; % Automatic Gain Control, or 'AGC Fast Attack' 
            % sdrReceiver.Gain = sdrCfg.rxGain; % if fixed gain, else AGC takes over

            if ~strcmpi(sdrCfg.deviceName, "Pluto")
                sdrReceiver.ShowAdvancedProperties = true;
                sdrReceiver.BypassUserLogic = true;
            end

            sdrReceiver.SamplesPerFrame = 2 * length(txWaveform);

        case 'simulated'
            sdrReceiver = dsp.SignalSource(sdrCfg.simulatedData);
        otherwise
            error('Unknown channel type: %s', sdrCfg.channel);
    end
end
