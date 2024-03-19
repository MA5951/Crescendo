package com.ma5951.utils.Logger;


import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class MALog {
    private static MALog logger;
    private final DataLog WPIlogger;
    private NetworkTable table;
    private final NetworkTableInstance networkTable;
    

    public MALog() {
        DataLogManager.start();
        WPIlogger = DataLogManager.getLog();
        networkTable = NetworkTableInstance.getDefault();
    }

    public NetworkTable getMainNT() {
        table = networkTable.getTable("/");
        return table;
    }

    public void logDriverStation() {
        DriverStation.startDataLog(DataLogManager.getLog());
    }

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
