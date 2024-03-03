package com.ma5951.utils.Logger;


import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

import edu.wpi.first.util.datalog.DataLog;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class MALog {
    private static MALog logger;
    private final DataLog WPIlogger;
    
    /*
     * Starts Logging the NT and the driver station data.
     */
    public MALog() {
        DataLogManager.start();
        WPIlogger = DataLogManager.getLog();
        DriverStation.startDataLog(DataLogManager.getLog());
    }


    /*
     * Stop Logging.
     */
    public void stopLog() {
        DataLogManager.stop();
    }

    public static MALog getInstance() {
        if (logger == null) {
            logger = new MALog();
        }
        return logger;
    }
    
}
