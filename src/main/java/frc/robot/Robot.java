// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.commands.DefaultRunInternallyControlledSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.swerve.DriveSwerveCommand;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    UpperShooter.getInstance();
    LowerShooter.getInstance();
    Intake.getInstance();
    Elevator.getInstance();
    LED.getInstance();

  CommandScheduler.getInstance().setDefaultCommand(
    UpperShooter.getInstance(), new DefaultRunInternallyControlledSubsystem(
      UpperShooter.getInstance(), ShooterConstants.defaultVUp));
    
  CommandScheduler.getInstance().setDefaultCommand(
    LowerShooter.getInstance(), new DefaultRunInternallyControlledSubsystem(
      LowerShooter.getInstance(), ShooterConstants.defaultVDown));

  CameraServer.startAutomaticCapture();


  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (DriverStation.isEnabled()) {
      RobotContainer.APRILTAGS_LIMELIGHT.periodic();
    }

    SmartDashboard.putBoolean("isFloor",
      RobotContainer.intakepose == RobotContainer.IntakePose.FLOOR);
    
    SmartDashboard.putBoolean("shooting linked to speaker",
      RobotContainer.ShootingLinkedToSpeaker);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    SwerveDrivetrainSubsystem.getInstance().resetEncoders();
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

    Elevator.getInstance().setSetPoint(Elevator.getInstance().getSetPoint());
    
    CommandScheduler.getInstance().setDefaultCommand(
      Elevator.getInstance(), new DefaultRunInternallyControlledSubsystem(
        Elevator.getInstance(), 0));

    CommandScheduler.getInstance().setDefaultCommand(
      SwerveDrivetrainSubsystem.getInstance(), 
      new DriveSwerveCommand(
        RobotContainer.driverController::getLeftX,
        RobotContainer.driverController::getLeftY,
        RobotContainer.driverController::getRightX));

  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("intake running", 
      RobotContainer.isIntakeRunning);
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
