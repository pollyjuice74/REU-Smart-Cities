function dataCfg = data_config()
    dataCfg.dataSource = "random"; % Options: "random", "ros"
    dataCfg.scaleFactor = 100;
    
    dataCfg.txDataBits = [];
    dataCfg.txLidarBytes = [];
    dataCfg.txLidarMatrix = [];

    dataCfg.rxDataBits = [];
    dataCfg.rxLidarBytes = [];
    dataCfg.rxLidarMatrix = [];
    
end
