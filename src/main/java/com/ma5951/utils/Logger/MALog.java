package com.ma5951.utils.Logger;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class MALog {
    private static MALog logger;
    private final DataLog log;



    public MALog() {
        DataLogManager.start();
        log = DataLogManager.getLog();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    

   
    public static MALog getInstance() {
        if (logger == null) {
            logger = new MALog();
        }
        return logger;
    }
    
}
