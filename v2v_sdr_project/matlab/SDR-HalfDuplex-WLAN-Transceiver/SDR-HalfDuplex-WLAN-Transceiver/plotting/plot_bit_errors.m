function [ber, bitErrors] = plot_bit_errors(txBits, rxBits, rows)
    %PLOT_BIT_ERRORS Visualize bit errors between txBits and rxBits as a heatmap
    %
    % Inputs:
    %   txBits - Transmitted bitstream (logical vector)
    %   rxBits - Received bitstream (logical vector)
    %   rows   - Number of rows for reshaping (must divide total length)

    % Reshape and truncate bits
    cols = floor(length(txBits) / rows);
    N = rows * cols;
    txMatrix = reshape(txBits(1:N), rows, cols);
    rxMatrix = reshape(rxBits(1:N), rows, cols);
    
    % Compute error matrix
    errorMatrix = txMatrix ~= rxMatrix;

    % BER calculation
    bitErrors = sum(errorMatrix(:));
    ber = bitErrors / N;
    fprintf('[WARNING] %d bits checked. %d bit errors (BER = %.2e).\n', N, bitErrors, ber);

    % Display image
    %figure('Name','Bit Error Map','NumberTitle','off');
    %imshow(errorMatrix, 'InitialMagnification', 'fit');
    %colormap([1 1 1; 1 0 0]);  % white = correct, red = error
    %title(sprintf('Bit Error Map (%d errors)', bitErrors));
end
