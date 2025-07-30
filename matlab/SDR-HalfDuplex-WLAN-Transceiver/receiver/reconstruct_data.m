function [dataCfg] = reconstruct_data(dataCfg, sdrCfg, waveCfg, viewers)
    
    % Variables
    msduLength = waveCfg.msduLength;
    numMSDUs = waveCfg.numMSDUs;

    txLidarMatrix = dataCfg.txLidarMatrix;
    scaleFactor = dataCfg.scaleFactor;
    packetSeq = dataCfg.packetSeq;
    pktOffset = dataCfg.pktOffset;
    fineTimingOffset = dataCfg.fineTimingOffset;

    txDataBits = dataCfg.txDataBits;
    txLidarBytes = dataCfg.txLidarBytes;
    
    rxDataBits = dataCfg.rxDataBits;
 
    % === Reconstruct Lidar Data ===
    if ~(isempty(fineTimingOffset) || isempty(pktOffset))

        % Convert decoded bits from cell array to column vector
        % Remove any extra bits
        rxDataBits = rxDataBits(1:end-(mod(length(rxDataBits),msduLength*8)));
        % Reshape such that each column length has bits equal to msduLength*8
        rxDataBits = reshape(rxDataBits,msduLength*8,[]);

        % Remove duplicate packets if any. Duplicate packets are located at the
        % end of rxDataBits
        if length(packetSeq)>numMSDUs
            numDupPackets = size(rxDataBits,2)-numMSDUs;
            rxDataBits = rxDataBits(:,1:end-numDupPackets);
        end

        % Initialize variables for while loop
        startSeq = [];
        i=-1;

        % Only execute this if one of the packet sequence values have been decoded
        % accurately
        if any(packetSeq<numMSDUs)
            while isempty(startSeq)
                % This searches for a known packetSeq value
                i = i + 1;
                startSeq = find(packetSeq==i);
            end
            % Circularly shift data so that received packets are in order for image reconstruction. It
            % is assumed that all packets following the starting packet are received in
            % order as this is how the image is transmitted.
            rxDataBits = circshift(rxDataBits,[0 -(startSeq(1)-i-1)]); % Order MAC fragments

            % Perform bit error rate (BER) calculation on reordered data
            bitErrorRate = comm.ErrorRate;
            err = bitErrorRate(double(rxDataBits(:)), ...
                txDataBits(1:length(reshape(rxDataBits,[],1))));

            if sdrCfg.displayFlag == true 
                fprintf('  \nBit Error Rate (BER):\n');
                fprintf('          Bit Error Rate (BER) = %0.5f\n',err(1));
                fprintf('          Number of bit errors = %d\n',err(2));
                fprintf('    Number of transmitted bits = %d\n\n',length(txDataBits));

                % === Log Statistics (Post-Reconstruction) ===
                dataCfg.berHistory(end+1) = err(1);
                dataCfg.bitsReceived(end+1) = length(txDataBits);
            
                viewers.logStats(dataCfg); % centralized logger
            end
        end

        % ===== rxLiDAR Plotting =====
        % Convert bits to uint8
        decData = bit2int(reshape(rxDataBits(:),8,[]),8,false)';

        % Pad if data is missing
        if length(decData) < length(txLidarBytes)
            numMissing = length(txLidarBytes) - length(decData);
            decData = [decData; zeros(numMissing, 1)];
        else
            decData = decData(1:length(txLidarBytes));
        end

        % Convert back to int16
        rxLidarInt = typecast(uint8(decData), 'int16');

        % Reconstruct the original [x y z i] or [x y z]
        lidarShape = size(txLidarMatrix); % from transmitter
        rxLidarMatrix = double(reshape(rxLidarInt, lidarShape)) / scaleFactor;
        dataCfg.rxLidarMatrix = rxLidarMatrix;
    end
    
end
