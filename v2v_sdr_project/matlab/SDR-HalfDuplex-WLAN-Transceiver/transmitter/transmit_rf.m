function transmit_rf(sdrTransmitter, txWaveform)
    fprintf('\nTransmitting WLAN waveform...\n');

    powerScaleFactor = 0.8;
    txWaveform = txWaveform .* (1/max(abs(txWaveform)) * powerScaleFactor);

    fprintf('[TX] Transmitting waveform...\n');
    transmitRepeat(sdrTransmitter, txWaveform);
end
