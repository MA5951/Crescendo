// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Logger;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

/** Add your docs here. */
public class LoggedInt {

    private IntPublisher loggedNum;
    private int lastNum;

    public LoggedInt(NetworkTable networkTable , String name) {
        loggedNum = networkTable.getDoubleTopic(name).publish();
        loggedNum.set(0);
        lastNum = 0;
    }

    public void updateNum(int num) {
        if (lastNum != num) {
            loggedNum.set(num);
            lastNum = num;
        }   

    }
}
