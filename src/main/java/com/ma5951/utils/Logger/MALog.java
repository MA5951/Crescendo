package com.ma5951.utils.Logger;


import edu.wpi.first.util.datalog.DataLog;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class MALog {
    private static MALog logger;
    private final DataLog WPIlogger;

    public MALog() {
        DataLogManager.start();
        WPIlogger = DataLogManager.getLog();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    public static MALog getInstance() {
        if (logger == null) {
            logger = new MALog();
        }
        return logger;
    }
    
}
