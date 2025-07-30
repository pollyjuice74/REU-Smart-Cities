% rx_main.m – Receiver entry point
function [dataCfg, sdrReceiver] = rx_main(txWaveform, sdrReceiver, dataCfg, nonHTcfg, sdrCfg, waveCfg, viewers)
    try 
        [rxWaveform] = receive_rf(txWaveform, sdrReceiver, sdrCfg, waveCfg);  % case logic for simulated/real transmission
        dataCfg.rxWaveform = rxWaveform
        
        % === Process Received Signal ===
        [dataCfg] = process_received_signal(rxWaveform, dataCfg, waveCfg, sdrCfg, nonHTcfg, viewers);
    
        % === Reconstruct Data ===
        if ~isempty(rxWaveform) % not rx wf is empty
            dataCfg = reconstruct_data(dataCfg, sdrCfg, waveCfg, viewers);
    
            % === Publish to ROS === 
            if dataCfg.dataSource == "ros"
                publish_to_ros(dataCfg); %  create this
            end
        end
end