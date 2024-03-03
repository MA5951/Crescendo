// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Logger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;

/** Add your docs here. */
public class LoggedBool {

    private BooleanPublisher loggedBool;
    private boolean lastBool;

    public LoggedBool(NetworkTable networkTable , String name) {
        loggedBool = networkTable.getBooleanTopic(name).publish();
        loggedBool.set(false);
        lastBool = false;
    }

    public void updateNum(Boolean bool) {
        if (lastBool != bool) {
            loggedBool.set(bool);
            lastBool = bool;
        }   

    }
}
