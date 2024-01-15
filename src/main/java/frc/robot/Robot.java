// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.commands.DefaultRunInternallyControlledSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.swerve.DriveSwerveCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private double time = -5;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SwerveDrivetrainSubsystem.getInstance();
    Intake.getInstance();

    CommandScheduler.getInstance().setDefaultCommand(
      Elevator.getInstance(), new DefaultRunInternallyControlledSubsystem(
        Elevator.getInstance(), ElevatorConstants.defaultPose));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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

    CommandScheduler.getInstance().setDefaultCommand(
      SwerveDrivetrainSubsystem.getInstance(), 
      new DriveSwerveCommand(
        RobotContainer.driverController::getLeftX,
        RobotContainer.driverController::getLeftY,
        RobotContainer.driverController::getRightX));
  }

  @Override
  public void teleopPeriodic() {
    if (DriverStation.isEnabled() && time == -5) {
      time = Timer.getFPGATimestamp();
    } else if (DriverStation.isDisabled()) {
      time = -5;
    }

    // start of endgame (20 seconds left)
    if ((time > 115 && time < 115.2) || (time > 115.4 && time < 115.6) || (time > 115.8 && time < 91)) {
      RobotContainer.operatorController.getHID().setRumble(RumbleType.kBothRumble, 1);
      RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
    } else {
      RobotContainer.operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
      RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);  
    }

    // last 3 seconds of match
    if ((time > 132 && time < 132.3) || (time > 133 && time < 133.3) || (time > 134 && time < 135)) {
      RobotContainer.operatorController.getHID().setRumble(RumbleType.kLeftRumble, 1);
      RobotContainer.driverController.getHID().setRumble(RumbleType.kLeftRumble, 1);
    } else {
      RobotContainer.operatorController.getHID().setRumble(RumbleType.kLeftRumble, 0);
      RobotContainer.driverController.getHID().setRumble(RumbleType.kLeftRumble, 0);  
    }
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
