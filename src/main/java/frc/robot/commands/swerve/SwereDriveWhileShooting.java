// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class SwereDriveWhileShooting extends Command {
  private SwerveDrivetrainSubsystem swerve;
  private double angle;
  private PIDController pid;

  public SwereDriveWhileShooting() {
      swerve = SwerveDrivetrainSubsystem.getInstance();
      addRequirements(swerve);


      pid = new PIDController(
      SwerveConstants.THATA_KP,
      SwerveConstants.THATA_KI,
      SwerveConstants.THATA_KD
      );
      angle = angle;
      pid.setTolerance(SwerveConstants.ANGLE_PID_TOLORANCE);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = SwerveDrivetrainSubsystem.getInstance().isXYReversed ?
     RobotContainer.driverController.getLeftY() : RobotContainer.driverController.getLeftX();
    double ySpeed = SwerveDrivetrainSubsystem.getInstance().isXYReversed ?
     RobotContainer.driverController.getLeftX() : RobotContainer.driverController.getLeftY();
    
     double turningSpeed = RobotContainer.driverController.getRightX();

    xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : xSpeed;
    ySpeed = Math.abs(ySpeed) < 0.25 ? 0 : ySpeed;

    turningSpeed = (Math.abs(turningSpeed) < 0.25 ? 
    angle = DriverStation.getAlliance().get() == Alliance.Blue ?
    angle - Math.PI : angle 
    : turningSpeed) * -1;

    xSpeed = xSpeed *
        swerve.maxVelocity *
        (SwerveDrivetrainSubsystem.getInstance().isXReversed ? -1 : 1);
    ySpeed = ySpeed *
        swerve.maxVelocity *
        (SwerveDrivetrainSubsystem.getInstance().isYReversed ? -1 : 1);
    turningSpeed = turningSpeed *
        swerve.maxAngularVelocity;

    swerve.drive(xSpeed, ySpeed, turningSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
