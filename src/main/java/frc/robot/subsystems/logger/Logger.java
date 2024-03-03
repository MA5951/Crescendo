// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.logger;

import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Logger.LoggedSwerveStates;
import com.ma5951.utils.Logger.LoggerTab;
import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class Logger extends SubsystemBase {
  /** Creates a new Logger. */
  private static Logger logger;
  private MALog log;
  private LoggerTab swerveTab;
  private LoggedDouble swerveGyroAngle;
  private LoggedDouble swerveOffsetAngle;
  private LoggedPose2d swervePosition;
  private LoggedSwerveStates swerveSwerveStates;


  public Logger() {
    log = new MALog();
    swerveTab = new LoggerTab("Swerve");
    swerveGyroAngle = new LoggedDouble(swerveTab.getTab(), "Gyro Angle");
    swerveOffsetAngle = new LoggedDouble(swerveTab.getTab(), "Fused Angle");
    swervePosition = new LoggedPose2d(swerveTab.getTab(), "Pose");
    swerveSwerveStates = new LoggedSwerveStates(swerveTab.getTab(), "Moudule States");
    

    
  }

  public static Logger getInstance() {
    if (logger == null) {
      logger = new Logger();
    }
    return logger;
  }

  @Override
  public void periodic() {
    swerveGyroAngle.update(SwerveDrivetrainSubsystem.getInstance().getFusedHeading());
    swerveOffsetAngle.update(SwerveDrivetrainSubsystem.getInstance().getOffsetAngle());
    swervePosition.update(SwerveDrivetrainSubsystem.getInstance().getPose());
    swerveSwerveStates.update(SwerveDrivetrainSubsystem.getInstance().getModuleStates());
  }
}
