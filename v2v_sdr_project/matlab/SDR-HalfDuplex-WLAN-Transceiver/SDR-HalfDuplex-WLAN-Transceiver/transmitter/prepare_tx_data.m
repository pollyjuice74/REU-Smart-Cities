function [dataCfg, waveCfg] = prepare_tx_data(dataCfg, waveCfg)
    % Variables
    msduLength = waveCfg.msduLength;
    txLidarBytes = dataCfg.txLidarBytes;
    
    % Fragmentation parameters
    numMSDUs = ceil(length(txLidarBytes) / msduLength);
    padZeros = msduLength * numMSDUs - length(txLidarBytes); % ensure full multiple
    txData = [txLidarBytes; zeros(padZeros,1)];

    % Convert to bits
    txDataBits = double(int2bit(txData, 8, false));

    % MAC fragmenting into MPDUs
    data = zeros(0,1);
    for i = 0:numMSDUs-1    
        % Extract image data (in octets) for each MPDU
        frameBody = txData(i*msduLength+1:msduLength*(i+1),:);

        % Create MAC frame configuration object and configure sequence number
        cfgMAC = wlanMACFrameConfig(FrameType='Data',SequenceNumber=i);

        % Generate MPDU
        [psdu, lengthMPDU]= wlanMACFrame(frameBody,cfgMAC,OutputFormat='bits');

        % Concatenate PSDUs for waveform generation
        data = [data; psdu]; %#ok<AGROW>
    end

    txDataBits = data; % Rename for clarity

    % Set variables
    waveCfg.numMSDUs = numMSDUs;
    waveCfg.lengthMPDU = lengthMPDU;

    dataCfg.txDataBits = txDataBits;
end
