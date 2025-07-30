function logger = make_metrics_logger(viewers)
    logger = @logStats;

    function logStats(dataCfg)
        % Variables
        berHistory = dataCfg.berHistory;
        bitsReceived = dataCfg.bitsReceived; 

        % Update the BER plot
        if isfield(viewers, 'berLine') && isvalid(viewers.berLine)
            set(viewers.berLine, 'YData', berHistory, 'XData', 1:length(berHistory));
    
            ylim(viewers.ax1, [0 max(1e-5, max(berHistory)*1.2)]);
            drawnow limitrate;
        end

        % Print summary
        fprintf('\n[Statistics Summary]\n');
        fprintf('Mean BER: %.5f | Std: %.5f | Mean Bits Received: %d | Packages Received: %d | Total Bits Received: %d \n', ...
            mean(berHistory), std(berHistory), mean(bitsReceived), length(bitsReceived), sum(bitsReceived));
        
        % Stop at 10,000,000 bits received for fair comparison 
        if sum(bitsReceived) > 10000000

            % Save file as .json
            if ~exist('recordings', 'dir')
                mkdir('recordings');
            end
            filename = sprintf('recordings/packetStats_%s.json', datestr(now, 'yyyymmdd_HHMMSS'));
            % data
            logData = struct( ...
                'timestamp', datestr(now, 'yyyy-mm-dd HH:MM:SS'), ...
                'meanBER', mean(berHistory), ...
                'stdBER', std(berHistory), ...
                'meanSentBits', mean(bitsReceived), ...
                'packagesReceived', length(bitsReceived), ...
                'totalBits', sum(bitsReceived) ...
            );
            jsonText = jsonencode(logData);
            fid = fopen(filename, 'w');
            fwrite(fid, jsonText, 'char');
            fclose(fid);
            fprintf('[✓] Stats saved to JSON: %s\n', filename);
 
            % then stop the entire main() program
            exit;   
        end
    end
end
