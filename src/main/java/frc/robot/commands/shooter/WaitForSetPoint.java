// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;
import frc.robot.subsystems.swerve.SwerveConstants;

public class WaitForSetPoint extends Command {
  /** Creates a new WaitForSetPoint. */
  private double lastTimeNotAtSetPointUpper = 0;
  private UpperShooter upper;
  private LowerShooter lower;


  public WaitForSetPoint() {
    upper = UpperShooter.getInstance();
    lower = LowerShooter.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(upper.atPoint() && lower.atPoint())) {
      lastTimeNotAtSetPointUpper = Timer.getFPGATimestamp();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - lastTimeNotAtSetPointUpper > ShooterConstants.TIME_AT_SETPOINT;
  }
}
