// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.AutoAutomations;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.AngleAdjust;
import frc.robot.commands.swerve.DriveSwerveCommand;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class ShootInMotion extends Command {
  /** Creates a new ShootInMotion. */
  private SwerveDrivetrainSubsystem swerve;
  private Command swerveCommand;
  private Double angle;

  public ShootInMotion() {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    addRequirements(swerve);
  }

  public void swerveDrive() {
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

  public void setShooter() {
    LowerShooter.getInstance().setSetPoint(
      LowerShooter.getInstance().getVelocityForShooting());

    UpperShooter.getInstance().setSetPoint(
      UpperShooter.getInstance().getVelocityForShooting());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.FactorVelocityTo(0.4);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive();
    setShooter();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.FactorVelocityTo(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.getPose().get;
  }
}
