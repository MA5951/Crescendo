// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.commands.DefaultRunInternallyControlledSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.swerve.DriveSwerveCommand;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // private double time = -5;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    UpperShooter.getInstance();
    LowerShooter.getInstance();
    Intake.getInstance();
    Elevator.getInstance();
    LED.getInstance();
    
  CommandScheduler.getInstance().setDefaultCommand(
    LowerShooter.getInstance(), new DefaultRunInternallyControlledSubsystem(
      LowerShooter.getInstance(), ShooterConstants.defaultV));

  CommandScheduler.getInstance().setDefaultCommand(
    UpperShooter.getInstance(), new DefaultRunInternallyControlledSubsystem(
      UpperShooter.getInstance(), ShooterConstants.defaultV));

    CommandScheduler.getInstance().setDefaultCommand(
      Elevator.getInstance(), new DefaultRunInternallyControlledSubsystem(
        Elevator.getInstance(), ElevatorConstants.DEFAULT_POSE));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (DriverStation.isEnabled()) {
      RobotContainer.APRILTAGS_LIMELIGHT.periodic();
    }

    SmartDashboard.putBoolean("isFloor",
      RobotContainer.intakepose == RobotContainer.IntakePose.FLOOR);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SwerveDrivetrainSubsystem.getInstance().resetEncoders();
    
    CommandScheduler.getInstance().setDefaultCommand(
      SwerveDrivetrainSubsystem.getInstance(), 
      new DriveSwerveCommand(
        RobotContainer.driverController::getLeftX,
        RobotContainer.driverController::getLeftY,
        RobotContainer.driverController::getRightX));

    // time = Timer.getFPGATimestamp();

  }

  @Override
  public void teleopPeriodic() {
    // double timePassed = Timer.getFPGATimestamp() - time;
    // start of endgame (20 seconds left)
    

    // if ((timePassed > 115 && timePassed < 115.2) || (timePassed > 115.4 && timePassed < 115.6) || (timePassed > 115.8 && timePassed < 116)) {
    //   RobotContainer.operatorController.getHID().setRumble(RumbleType.kBothRumble, 1);
    //   RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
    // } else {
    //   RobotContainer.operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
    //   RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);  
    // }

    // // last 3 seconds of match
    // if ((timePassed > 132 && timePassed < 132.3) || (timePassed > 133 && timePassed < 133.3) || (timePassed > 134 && timePassed < 134.3)) {
    //   RobotContainer.operatorController.getHID().setRumble(RumbleType.kLeftRumble, 1);
    //   RobotContainer.driverController.getHID().setRumble(RumbleType.kLeftRumble, 1);
    // } else {
    //   RobotContainer.operatorController.getHID().setRumble(RumbleType.kLeftRumble, 0);
    //   RobotContainer.driverController.getHID().setRumble(RumbleType.kLeftRumble, 0);  
    // }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
