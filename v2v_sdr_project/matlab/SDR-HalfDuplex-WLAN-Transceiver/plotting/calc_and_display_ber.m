function [ber, bitErrors, N] = calc_and_display_ber(dataCfg, viewers)
    N = min(length(dataCfg.txDataBits), length(dataCfg.rxDataBits));
    txBits = dataCfg.txDataBits(1:N);
    rxBits = dataCfg.rxDataBits(1:N);

    bitErrors = sum(txBits ~= rxBits);
    ber = bitErrors / N;

    % Update bit error heatmap plot without recreating figure
    update_bit_error_plot(txBits, rxBits, viewers, 100);  % rows=100

    % Optionally update other viewers here (spectrum, constellation, etc)
    % You can add calls here to update those with latest waveform data if available
end
