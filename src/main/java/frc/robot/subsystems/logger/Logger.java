// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.logger;

import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedInt;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Logger.LoggedSwerveStates;
import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class Logger extends SubsystemBase {
  /** Creates a new Logger. */
  private static Logger logger;
  private MALog log;
  private PowerDistribution pdh;
  private SwerveDrivetrainSubsystem swerve;
  
  //Power and can Logged Variables
  private LoggedDouble powerAndCanVoltage;
  private LoggedDouble powerAndCanAmpere;
  private LoggedInt powerAndCanCanUtalisation;


  //Swerve Logged Variables
  private LoggedDouble swerveGyroYaw;
  private LoggedDouble swerveGyroRoll;
  private LoggedDouble swerveGyroPitch;
  private LoggedDouble swerveSpeed;
  private LoggedDouble swerveAccleration;
  private LoggedDouble swerveFrontLeftCurrent;
  private LoggedDouble swerveFrontRightCurrent;
  private LoggedDouble swerveRearLeftCurrent;
  private LoggedDouble swerveRearRightCurrent;
  private LoggedPose2d swervePose;
  private LoggedSwerveStates swerveModulesStates;
  


  public Logger() {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    pdh = new PowerDistribution(PortMap.Robot.PDH, ModuleType.kRev);
    log = new MALog();

    


    swerveGyroYaw = new LoggedDouble("/Swerve/Gyro/Yaw");
    swerveGyroRoll = new LoggedDouble("/Swerve/Gyro/Roll");
    swerveGyroPitch = new LoggedDouble("/Swerve/Gyro/Pitch");
    swerveSpeed = new LoggedDouble("/Swerve/Speed");
    swerveAccleration = new LoggedDouble("/Swerve/Acceleration");
    swervePose = new LoggedPose2d("/Swerve/Robot Pose");
    swerveModulesStates = new LoggedSwerveStates("/Swerve/Modules States");
    swerveFrontLeftCurrent = new LoggedDouble("/Swerve/Currents/Front Left");
    swerveFrontRightCurrent = new LoggedDouble("/Swerve/Currents/Front Right");
    swerveRearLeftCurrent = new LoggedDouble("/Swerve/Currents/Rear Left");
    swerveRearRightCurrent = new LoggedDouble("/Swerve/Currents/Rear Right");

    powerAndCanVoltage = new LoggedDouble("/Power And Can/Voltage");
    powerAndCanAmpere = new LoggedDouble("/Power And Can/Ampere");
    powerAndCanCanUtalisation = new LoggedInt("/Power And Can/Can Utalisation");



 
   
    

    
  }

  public static Logger getInstance() {
    if (logger == null) {
      logger = new Logger();
    }
    return logger;
  }

  @Override
  public void periodic() {
    //Power And Can Update
    powerAndCanVoltage.update(pdh.getVoltage());
    powerAndCanAmpere.update(pdh.getTotalCurrent());
    powerAndCanCanUtalisation.update(0);// TODO


    //Swerve Log Update
    swerveGyroYaw.update(swerve.getFusedHeading());
    swerveGyroRoll.update(swerve.getRoll());
    swerveGyroPitch.update(swerve.getPitch());
    swerveSpeed.update(swerve.getVelocity());
    swerveAccleration.update(swerve.getAcceleration());
    swervePose.update(swerve.getPose());
    swerveModulesStates.update(swerve.getModulesStates());
    swerveFrontLeftCurrent.update(swerve.getModulesCurrent()[0]);
    swerveFrontRightCurrent.update(swerve.getModulesCurrent()[1]);
    swerveRearLeftCurrent.update(swerve.getModulesCurrent()[2]);
    swerveRearRightCurrent.update(swerve.getModulesCurrent()[3]);

  }
}
