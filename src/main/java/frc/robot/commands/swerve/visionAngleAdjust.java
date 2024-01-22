// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class visionAngleAdjust extends Command {
  /** Creates a new visionAngleAdjust. */
  private Supplier<Double> visionAngleSupplier;
  private SwerveDrivetrainSubsystem swerve;
  private PIDController pid;

  public visionAngleAdjust(Supplier<Double> visionAngleSupplier, double setpoint) {
    this.visionAngleSupplier = visionAngleSupplier;
    swerve = SwerveDrivetrainSubsystem.getInstance();
    pid = new PIDController(SwerveConstants.visionKp,
      SwerveConstants.visionKi, SwerveConstants.visionKd);
    pid.setTolerance(SwerveConstants.visionTolorance);
    pid.setSetpoint(setpoint);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(0, 0, pid.calculate(visionAngleSupplier.get()), false);
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
