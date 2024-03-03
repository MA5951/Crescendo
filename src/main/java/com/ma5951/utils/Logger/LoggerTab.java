// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LoggerTab {
    
    private NetworkTable table;
    private final NetworkTableInstance networkTable;


    public LoggerTab(String name) {
        networkTable = NetworkTableInstance.getDefault();
        table = networkTable.getTable(name);    
    }

    public NetworkTable getTab() {
        return table;
    }

}
