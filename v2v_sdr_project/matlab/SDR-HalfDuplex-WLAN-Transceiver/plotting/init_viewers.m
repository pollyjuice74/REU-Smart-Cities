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
    refQAM = wlanReferenceSymbols('64QAM');
    viewers.constellation = comm.ConstellationDiagram( ...
        'Title', 'Equalized WLAN Symbols', ...
        'ShowReferenceConstellation', true, ...
        'ReferenceConstellation', refQAM, ...
        'Position', [878 376 460 460]);

    % === Image Figure Handle ===
    viewers.imFig = imFig;
end

% % Visualize point cloud
% fprintf('\nConstructing point cloud from received data.\n');
% pcshow(rxLidarMatrix(:,1:3)); % 3D plot
% title('Received LiDAR Point Cloud');

% figure;
% subplot(1,2,1);
% pcshow(txLidarMatrix(:,1:3));
% title('Original LiDAR');
% 
% subplot(1,2,2);
% pcshow(rxLidarMatrix(:,1:3));
% title('Received LiDAR');