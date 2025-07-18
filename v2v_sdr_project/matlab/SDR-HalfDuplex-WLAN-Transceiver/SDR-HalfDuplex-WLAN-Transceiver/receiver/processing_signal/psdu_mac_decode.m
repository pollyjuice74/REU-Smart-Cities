function [dataCfg] = psdu_mac_decode(rxWaveform, dataCfg, waveCfg, nonHTcfg, sdrCfg, viewers)
    % === Variables ===
    chanBW = nonHTcfg.ChannelBandwidth;

    % === Set up required variables for receiver processing ===
    rxWaveformLen = size(rxWaveform,1);
    searchOffset = 0; % Offset from start of the waveform in samples

    % === Get the required field indices within a PSDU ===
    ind = wlanFieldIndices(nonHTcfg);
    Ns = ind.LSIG(2)-ind.LSIG(1)+1; % Number of samples in an OFDM symbol

    % Minimum packet length is 10 OFDM symbols
    lstfLen = double(ind.LSTF(2)); % Number of samples in L-STF
    minPktLen = lstfLen*5;
    pktInd = 1;
    fineTimingOffset = [];
    packetSeq = [];
    rxBit = [];
    % Divide input data stream into fragments
    bitsPerOctet = 8;

    % Perform EVM calculation
    evmCalculator = comm.EVM(AveragingDimensions=[1 2 3]);
    evmCalculator.MaximumEVMOutputPort = true;

    % === Process the received out-of-order packets ===
    while (searchOffset+minPktLen)<=rxWaveformLen
        % Packet detect
        pktOffset = wlanPacketDetect(rxWaveform,chanBW,searchOffset,0.5); % lower threshold improves detection under Doppler, default correlation value was 0.5

        % Adjust packet offset
        pktOffset = searchOffset+pktOffset;
        if isempty(pktOffset) || (pktOffset+double(ind.LSIG(2))>rxWaveformLen)
            if pktInd==1
                disp('** No packet detected **');
            end
            break;
        end

        % Extract non-HT fields and perform coarse frequency offset correction
        % to allow for reliable symbol timing
        nonHT = rxWaveform(pktOffset+(ind.LSTF(1):ind.LSIG(2)),:);
        coarseFreqOffset = wlanCoarseCFOEstimate(nonHT,chanBW);
        nonHT = frequencyOffset(nonHT, waveCfg.sampleRate,-coarseFreqOffset);

        % Symbol timing synchronization
        fineTimingOffset = wlanSymbolTimingEstimate(nonHT,chanBW);

        % Adjust packet offset
        pktOffset = pktOffset+fineTimingOffset;

        % Timing synchronization complete: Packet detected and synchronized
        % Extract the non-HT preamble field after synchronization and
        % perform frequency correction
        if (pktOffset<0) || ((pktOffset+minPktLen)>rxWaveformLen)
            searchOffset = pktOffset+1.5*lstfLen;
            continue;
        end
        fprintf('\nPacket-%d detected at index %d\n',pktInd,pktOffset+1);

        % Extract first 7 OFDM symbols worth of data for format detection and
        % L-SIG decoding
        nonHT = rxWaveform(pktOffset+(1:7*Ns),:);
        nonHT = frequencyOffset(nonHT, waveCfg.sampleRate,-coarseFreqOffset);

        % Perform fine frequency offset correction on the synchronized and
        % coarse corrected preamble fields
        lltf = nonHT(ind.LLTF(1):ind.LLTF(2),:);           % Extract L-LTF
        fineFreqOffset = wlanFineCFOEstimate(lltf,chanBW);
        nonHT = frequencyOffset(nonHT, waveCfg.sampleRate,-fineFreqOffset);
        cfoCorrection = coarseFreqOffset+fineFreqOffset; % Total CFO

        % Channel estimation using L-LTF
        lltf = nonHT(ind.LLTF(1):ind.LLTF(2),:);
        demodLLTF = wlanLLTFDemodulate(lltf,chanBW);
        chanEstLLTF = wlanLLTFChannelEstimate(demodLLTF,chanBW);

        % Noise estimation
        noiseVarNonHT = wlanLLTFNoiseEstimate(demodLLTF);

        % Packet format detection using the 3 OFDM symbols immediately
        % following the L-LTF
        format = wlanFormatDetect(nonHT(ind.LLTF(2)+(1:3*Ns),:), ...
            chanEstLLTF,noiseVarNonHT,chanBW);
        disp(['  ' format ' format detected']);
        if ~strcmp(format,'Non-HT')
            fprintf('  A format other than Non-HT has been detected\n');
            searchOffset = pktOffset+1.5*lstfLen;
            continue;
        end

        % Recover L-SIG field bits
        [recLSIGBits,failCheck] = wlanLSIGRecover( ...
            nonHT(ind.LSIG(1):ind.LSIG(2),:), ...
            chanEstLLTF,noiseVarNonHT,chanBW);

        if failCheck
            fprintf('  L-SIG check fail \n');
            searchOffset = pktOffset+1.5*lstfLen;
            continue;
        else
            fprintf('  L-SIG check pass \n');
        end

        % Retrieve packet parameters based on decoded L-SIG
        [lsigMCS,lsigLen,rxSamples] = helperInterpretLSIG(recLSIGBits, waveCfg.sampleRate);

        if (rxSamples+pktOffset)>length(rxWaveform)
            disp('** Not enough samples to decode packet **');
            break;
        end

        % Apply CFO correction to the entire packet
        rxWaveform(pktOffset+(1:rxSamples),:) = frequencyOffset(...
            rxWaveform(pktOffset+(1:rxSamples),:), waveCfg.sampleRate,-cfoCorrection);

        % Create a receive Non-HT config object
        rxNonHTcfg = wlanNonHTConfig;
        rxNonHTcfg.MCS = lsigMCS;
        rxNonHTcfg.PSDULength = lsigLen;

        % Get the data field indices within a PPDU
        indNonHTData = wlanFieldIndices(rxNonHTcfg,'NonHT-Data');

        % Recover PSDU bits using transmitted packet parameters and channel
        % estimates from L-LTF
        [rxPSDU,eqSym] = wlanNonHTDataRecover(rxWaveform(pktOffset+... % error likely due to packet construction
            (indNonHTData(1):indNonHTData(2)),:), ...
            chanEstLLTF,noiseVarNonHT,rxNonHTcfg);

        viewers.constellation(reshape(eqSym,[],1)); % Current constellation
        release(viewers.constellation);

        refSym = wlanClosestReferenceSymbol(eqSym,rxNonHTcfg);
        [evm.RMS,evm.Peak] = evmCalculator(refSym,eqSym);

        % Decode the MPDU and extract MSDU
        [cfgMACRx,msduList{pktInd},status] = wlanMPDUDecode(rxPSDU,rxNonHTcfg); %#ok<*SAGROW>

        if strcmp(status,'Success')
            disp('  MAC FCS check pass');

            % Store sequencing information
            packetSeq(pktInd) = cfgMACRx.SequenceNumber;

            % Convert MSDU to a binary data stream
            rxBit{pktInd} = int2bit(hex2dec(cell2mat(msduList{pktInd})),8,false);

        else % Decoding failed
            if strcmp(status,'FCSFailed')
                % FCS failed
                disp('  MAC FCS check fail');
            else
                % FCS passed but encountered other decoding failures
                disp('  MAC FCS check pass');
            end

            % Since there are no retransmissions modeled in this example, we
            % extract the image data (MSDU) and sequence number from the MPDU,
            % even though FCS check fails.

            % Remove header and FCS. Extract the MSDU.
            macHeaderBitsLength = 24*bitsPerOctet;
            fcsBitsLength = 4*bitsPerOctet;
            msduList{pktInd} = rxPSDU(macHeaderBitsLength+1:end-fcsBitsLength);

            % Extract and store sequence number
            sequenceNumStartIndex = 23*bitsPerOctet+1;
            sequenceNumEndIndex = 25*bitsPerOctet-4;
            conversionLength = sequenceNumEndIndex-sequenceNumStartIndex+1;
            packetSeq(pktInd) = bit2int(rxPSDU(sequenceNumStartIndex:sequenceNumEndIndex),conversionLength,false);

            % MSDU binary data stream
            rxBit{pktInd} = double(msduList{pktInd});
        end

        % Display decoded information
        if sdrCfg.displayFlag
            fprintf('  Estimated CFO: %5.1f Hz\n\n',cfoCorrection); %#ok<*UNRCH> 

            disp('  Decoded L-SIG contents: ');
            fprintf('                            MCS: %d\n',lsigMCS);
            fprintf('                         Length: %d\n',lsigLen);
            fprintf('    Number of samples in packet: %d\n\n',rxSamples);

            fprintf('  EVM:\n');
            fprintf('    EVM peak: %0.3f%%  EVM RMS: %0.3f%%\n\n', ...
                evm.Peak,evm.RMS);

            fprintf('  Decoded MAC Sequence Control field contents:\n');
            fprintf('    Sequence number: %d\n\n',packetSeq(pktInd));
        end

        % Update search index
        searchOffset = pktOffset+double(indNonHTData(2));
        
        % Finish processing when a duplicate packet is detected. The
        % recovered data includes bits from duplicate frame
        % Remove the data bits from the duplicate frame
        if length(unique(packetSeq)) < length(packetSeq)
            rxBit = rxBit(1:length(unique(packetSeq)));
            packetSeq = packetSeq(1:length(unique(packetSeq)));
            break
        end

        % === REASSEMBLE FULL BYTE STREAM ===
        % Convert cell array of bits to a single array
        allBits = [rxBit{:}];
        rxBytes = bit2int(reshape(allBits, 8, []).', 8, false);
        
        % Example: decode rxBytes to XYZ point cloud
        % Replace this with your actual decoding logic (e.g., reshape into float32 triples)
        if isempty(rxBytes)
            warning('No bytes to decode into point cloud.');
            continue;
        end

        disp('rxBytes:');
        disp(rxBytes);
        rxBytes = rxBytes(:);
        xyzMatrix = typecast(uint8(rxBytes), 'single');
        xyzMatrix = reshape(xyzMatrix, 3, []).'; % Nx3 matrix for pointCloud
        
        % % % === ROS PUBLISHER ===
        % if strcmp(dataCfg.dataSource, 'ros')
        %     % if ~robotics.ros.internal.Global.isNodeActive
        %     %     rosinit('http://<ros_master_ip>:11311'); % Replace with actual ROS master
        %     % end
        % 
        %     ptCloudPub = rospublisher('/lidar_rx', 'sensor_msgs/PointCloud2');
        %     msg = rosmessage(ptCloudPub);
        % 
        %     ptCloud = pointCloud(xyzMatrix);
        %     msg = rosWriteXYZ(msg, ptCloud);
        %     send(ptCloudPub, msg);
        % 
        %     disp('Published point cloud to ROS.');
        % end

        % Counter
        pktInd = pktInd+1;
    end

    % === Final Outputs ===
    % Flatten bitstream
    rxDataBits = cat(1, rxBit{:}); % Column vector
    % Convert to bytes
    rxLidarBytes = bit2int(reshape(rxDataBits, 8, []).', 8, false);

    % Set variables
    dataCfg.fineTimingOffset = fineTimingOffset
    dataCfg.packetSeq = packetSeq;
    dataCfg.pktOffset = pktOffset;

    dataCfg.rxDataBits = rxDataBits;
    dataCfg.rxLidarBytes = rxLidarBytes;
end
