// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class LockModules extends Command {
  private SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
  private final SwerveModuleState[] states;
  public LockModules() {
    swerveDrivetrainSubsystem = SwerveDrivetrainSubsystem.getInstance();
    addRequirements(swerveDrivetrainSubsystem);
    states = new SwerveModuleState[] {
      new SwerveModuleState(
        1e-41,
        Rotation2d.fromDegrees(-45)
      ),
      new SwerveModuleState(
        1e-41,
        Rotation2d.fromDegrees(45)
      ),
      new SwerveModuleState(
        1e-41,
        Rotation2d.fromDegrees(45)
      ),
      new SwerveModuleState(
        1e-41,
        Rotation2d.fromDegrees(-45)
      ),
    };
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerveDrivetrainSubsystem.setModules(states);
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
