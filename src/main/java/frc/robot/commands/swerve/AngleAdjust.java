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

  public AngleAdjust(Supplier<Double> angle) {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    addRequirements(swerve);

    pid = new PIDController(
      SwerveConstants.THATA_KP,
      SwerveConstants.THATA_KI,
      SwerveConstants.THATA_KD
    );
    this.angle = angle;
    pid.setTolerance(SwerveConstants.anglePIDTolorance);
  }

  public AngleAdjust(double angle) {
    this(() -> angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(angle.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(
      0, 0,
      pid.calculate(swerve.getPose().getRotation().getRadians())
      , false);
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
