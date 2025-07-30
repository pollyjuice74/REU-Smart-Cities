function viewers = init_viewers()
    % === Image Plot Viewer ===
    imFig = figure;
    imFig.NumberTitle = 'off';
    imFig.Name = 'Image Plot';
    imFig.Visible = 'off';
    
    % === Spectrum Viewer ===
    viewers.spectrumScope = spectrumAnalyzer( ...
        'SpectrumType', 'power-density', ...
        'Title', 'Received Baseband WLAN Signal Spectrum', ...
        'YLabel', 'Power spectral density', ...
        'Position', [69 376 800 450]);

    % === Constellation Viewer ===
    refQAM = wlanReferenceSymbols('16QAM');
    viewers.constellation = comm.ConstellationDiagram( ...
        'Title', 'Equalized WLAN Symbols', ...
        'ShowReferenceConstellation', true, ...
        'ReferenceConstellation', refQAM, ...
        'Position', [878 376 460 460]);

    % === Figure Handle ===
    viewers.fig = figure('Name', 'BER Viewer');

    % BER subplot
    viewers.ax1 = subplot(2, 1, 1);
    viewers.berLine = plot(viewers.ax1, NaN, NaN, 'r');
    title(viewers.ax1, 'BER Over Time');
    xlabel(viewers.ax1, 'Packet Index');
    ylabel(viewers.ax1, 'BER');
end
