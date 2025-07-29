function [rxWaveform] = receive_rf(txWaveform, sdrReceiver, sdrCfg, waveCfg)

    switch sdrCfg.channel
        case 'OverTheAir'
            fprintf('[RX] Capturing waveform...\n');
            rxWaveform = capture(sdrReceiver, sdrReceiver.SamplesPerFrame, 'Samples');

        case 'GaussianNoise'
            rxWaveform = awgn(txWaveform, sdrCfg.SNR, 'measured');
        case 'NoImpairments'
            rxWaveform = txWaveform;

        otherwise
            error('Unsupported channel type: %s', sdrCfg.channel);
    end
end
