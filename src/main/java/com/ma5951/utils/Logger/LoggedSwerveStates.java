// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructArrayPublisher;

/** Add your docs here. */
public class LoggedSwerveStates {

    private StructArrayPublisher<SwerveModuleState> loggedStates;
    private SwerveModuleState[] lastStates;

    public LoggedSwerveStates(NetworkTable networkTable , String name) {
        loggedStates = networkTable.getStructArrayTopic(name , SwerveModuleState.struct).publish();
        loggedStates.set(null);
        lastStates = null;
    }

    public void update(SwerveModuleState[] states) {
        if (lastStates != states) {
            loggedStates.set(states);
            lastStates = states;
        }   

    }
}
