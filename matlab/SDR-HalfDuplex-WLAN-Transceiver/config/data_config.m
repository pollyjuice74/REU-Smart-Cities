function dataCfg = data_config()
    dataCfg.dataSource = "random"; % Options: "random", "ros"
    dataCfg.scaleFactor = 100;
    dataCfg.subscriberPath = '/local_lidar';
    dataCfg.publisherPath = '/sdr_lidar';
    
    dataCfg.txDataBits = [];
    dataCfg.txLidarBytes = [];
    dataCfg.txLidarMatrix = [];

    dataCfg.rxDataBits = [];
    dataCfg.rxLidarBytes = [];
    dataCfg.rxLidarMatrix = [];

    dataCfg.berHistory = [];
    dataCfg.bitsReceived = [];

end
