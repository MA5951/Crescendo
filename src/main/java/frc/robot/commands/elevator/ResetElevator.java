// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ResetElevator extends Command {
  /** Creates a new ResetElevator. */
  private Elevator elevator;
  private double lastTimeWithNoJump = 0;

  public ResetElevator() {
    elevator = Elevator.getInstance();
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.getCurrent() < ElevatorConstants.CURRENT_THRESHOLD) {
      lastTimeWithNoJump = Timer.getFPGATimestamp();
    }
    elevator.setPower(ElevatorConstants.CLOSING_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      elevator.resetPose(0);
    }
    elevator.setPower(0);
    elevator.setSetPoint(ElevatorConstants.DEFAULT_POSE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - lastTimeWithNoJump > 
      ElevatorConstants.TIME_WITH_CURRENT_JUMP;
  }
}
