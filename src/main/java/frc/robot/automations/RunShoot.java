// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class RunShoot extends Command {
  private Shoot commandShoot;
  public RunShoot() {
    commandShoot = new Shoot();
  }

  @Override
  public void initialize() {
    commandShoot.initialize();
    System.out.println("start");
  }

  @Override
  public void execute() {
    commandShoot.execute();
  }

  @Override
  public void end(boolean interrupted) {
    commandShoot.end(interrupted);
    SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(1);
  }

  @Override
  public boolean isFinished() {
    return !SwerveDrivetrainSubsystem.getInstance().canShoot() || RobotContainer.isIntakeRunning;
  }
}
