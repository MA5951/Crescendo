// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.logger;

import com.ma5951.utils.Logger.LoggedNum;
import com.ma5951.utils.Logger.LoggerTab;
import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Logger extends SubsystemBase {
  /** Creates a new Logger. */
  private static Logger logger;
  private MALog log;
  private LoggerTab swerveTab;
  private LoggedNum swerveAngle;


  public Logger() {
    log = new MALog();
    swerveTab = new LoggerTab("Swerve");
    swerveAngle = new LoggedNum(swerveTab.getTable(), "Angle");
    
  }

  public static Logger getInstance() {
    if (logger == null) {
      logger = new Logger();
    }
    return logger;
  }

  @Override
  public void periodic() {
    swerveAngle.updateNum(10d);
  }
}
