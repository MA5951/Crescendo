// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.logger;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedInt;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Logger.LoggedString;
import com.ma5951.utils.Logger.LoggedSwerveStates;
import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class Logger extends SubsystemBase {
  /** Creates a new Logger. */
  private static Logger logger;
  private MALog log;
  private PowerDistribution pdh;
  private SwerveDrivetrainSubsystem swerve;
  private Intake intake;
  private Elevator elevator;
  private UpperShooter upperShooter;
  private LowerShooter lowerShooter;
  
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

  //Intake Logged Variables
  private LoggedDouble intakePower;
  private LoggedDouble intakeCurrent;
  private LoggedBool intakeUpperSensor;
  private LoggedBool intakeLowerSensor;
  private LoggedBool intakeIsRing;
  private LoggedBool intakeCanMove;

  //Elevator Logged Variables
  private LoggedDouble elevatorSetPoint;
  private LoggedDouble elevatorPose;
  private LoggedDouble elevatorCurrent;
  private LoggedBool elevatorAtPoint;
  private LoggedBool elevatorCanMove;

  //Upper Shooter Logged Variables
  private LoggedDouble uppershooterSetPoint;
  private LoggedDouble uppershooterSpeed;
  private LoggedBool uppershooterAtPoint;
  private LoggedString uppershooterIdleMode;

  public Logger() {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    intake = Intake.getInstance();
    upperShooter = UpperShooter.getInstance();
    lowerShooter = LowerShooter.getInstance();
    elevator = Elevator.getInstance();
    pdh = new PowerDistribution(PortMap.Robot.PDH, ModuleType.kRev);
    log = new MALog();

    

    //Swerve Logged Variables
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

    //Power and can Logged Variables
    powerAndCanVoltage = new LoggedDouble("/Power And Can/Voltage");
    powerAndCanAmpere = new LoggedDouble("/Power And Can/Ampere");
    powerAndCanCanUtalisation = new LoggedInt("/Power And Can/Can Utalisation");

    //Intake Logged Variables
    intakePower = new LoggedDouble("/Intake/Power");
    intakeCurrent = new LoggedDouble("/Intake/Current");
    intakeUpperSensor = new LoggedBool("/Intake/Upper Sensor");
    intakeLowerSensor = new LoggedBool("/Intake/Lower Sensor");
    intakeIsRing = new LoggedBool("/Intake/ Is Ring");
    intakeCanMove = new LoggedBool("/Intake/Can Move");

    //Elevator Logged Variables
    elevatorSetPoint = new LoggedDouble("/Elevator/Set Point");
    elevatorPose = new LoggedDouble("/Elevator/Pose");
    elevatorAtPoint = new LoggedBool("/Elevator/At Point");
    elevatorCurrent = new LoggedDouble("/Elevator/Currnt");
    elevatorCanMove = new LoggedBool("/Elevator/Can Move");

    //Upper Shooter Logged Variables
    uppershooterSetPoint = new LoggedDouble("/Upper Shooter/Set Point");
    uppershooterSpeed = new LoggedDouble("/Upper Shooter/Velocity");
    uppershooterAtPoint = new LoggedBool("/Upper Shooter/At Point");
    uppershooterIdleMode = new LoggedString("/Upper Shooter/Idle Mode");

    //Lower Shooter Logged Variables
    

  }

  public static Logger getInstance() {
    if (logger == null) {
      logger = new Logger();
    }
    return logger;
  }

  @Override
  public void periodic() {
    //Power And Can Updates
    powerAndCanVoltage.update(pdh.getVoltage());
    powerAndCanAmpere.update(pdh.getTotalCurrent());
    powerAndCanCanUtalisation.update(0);// TODO

    //Swerve Updates
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

    //Intake Updates
    intakePower.update(intake.getPower());
    intakeCurrent.update(intake.getCurrent());
    intakeUpperSensor.update(intake.getUpperSensore());
    intakeLowerSensor.update(intake.getLowerSensor());
    intakeIsRing.update(intake.isGamePieceInIntake());
    intakeCanMove.update(intake.canMove());

    //Elevator Updates
    elevatorSetPoint.update(elevator.getSetPoint());
    elevatorPose.update(elevator.getPosition());
    elevatorAtPoint.update(elevator.atPoint());
    elevatorCurrent.update(elevator.getCurrent());
    elevatorCanMove.update(elevator.canMove());

    //Upper Shooter Updates
    uppershooterSetPoint.update(upperShooter.getSetPoint());
    uppershooterSpeed.update(upperShooter.getVelocity());
    uppershooterAtPoint.update(upperShooter.atPoint());
    uppershooterIdleMode.update(upperShooter.getIDLmode());


  }
}
