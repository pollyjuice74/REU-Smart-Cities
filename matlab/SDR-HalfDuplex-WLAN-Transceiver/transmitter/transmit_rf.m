function transmit_rf(sdrTransmitter, txWaveform, transmitRepeat_flag)
    fprintf('\nTransmitting WLAN waveform...\n');

    powerScaleFactor = 0.8;
    txWaveform = txWaveform .* (1/max(abs(txWaveform)) * powerScaleFactor);

    % === Transmit waveform ===
    if transmitRepeat_flag
        fprintf('[TX] Transmitting in repeat mode...\n');
        transmitRepeat(sdrTransmitter, txWaveform);
    else
        fprintf('[TX] Transmitting once...\n');
        sdrTransmitter(txWaveform);
    end
end
