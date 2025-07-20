% tx_main.m – Transmitter entry point
function [txWaveform, sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg] = tx_main(sdrTransmitter, dataCfg, nonHTcfg, sdrCfg, waveCfg)
    % Prepare TX Data
    [dataCfg, waveCfg] = prepare_tx_data(dataCfg, waveCfg);  % Or modify prepare_tx_data() to accept bytes
    
    % Create PHY waveform config
    nonHTcfg = phy_config(waveCfg)

    % Generate waveform
    [txWaveform, nonHTcfg, sdrCfg, waveCfg] = generate_waveform(dataCfg, nonHTcfg, sdrCfg, waveCfg);

    % Transmit (optional)
    transmit_rf(sdrTransmitter, txWaveform);
end
