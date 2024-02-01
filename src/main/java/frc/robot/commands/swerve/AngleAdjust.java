// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class AngleAdjust extends Command {
  private SwerveDrivetrainSubsystem swerve;
  private PIDController pid;
  private Supplier<Double> angle;
  private Supplier<Double> xSupplier;
  private Supplier<Double> ySupplier;

  public AngleAdjust(Supplier<Double> angle,
    Supplier<Double> xSupplier, Supplier<Double> ySupplier) {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    addRequirements(swerve);

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    pid = new PIDController(
      SwerveConstants.THATA_KP,
      SwerveConstants.THATA_KI,
      SwerveConstants.THATA_KD
    );
    this.angle = angle;
    pid.setTolerance(SwerveConstants.ANGLE_PID_TOLORANCE);
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setSetpoint(angle.get());
    double xSpeed = SwerveDrivetrainSubsystem.getInstance().isXYReversed ? 
      ySupplier.get() : xSupplier.get();
    double ySpeed = SwerveDrivetrainSubsystem.getInstance().isXYReversed ?
      xSupplier.get() : ySupplier.get();
    xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : xSpeed;
    ySpeed = Math.abs(ySpeed) < 0.1 ? 0 : ySpeed;
    xSpeed = xSpeed *
      swerve.maxVelocity *
      (SwerveDrivetrainSubsystem.getInstance().isXReversed ? -1 : 1);
    ySpeed = ySpeed *
      swerve.maxVelocity *
      (SwerveDrivetrainSubsystem.getInstance().isYReversed ? -1 : 1);

    swerve.drive(
      xSpeed, ySpeed,
      pid.calculate(swerve.getPose().getRotation().getRadians())
      , true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
