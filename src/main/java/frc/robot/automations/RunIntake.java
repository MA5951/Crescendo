// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;

public class RunIntake extends Command {
  private Command intakeCommand;
  private boolean finished;

  public RunIntake(double power) {
    intakeCommand = new IntakeAndRingCenter(power);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = Intake.getInstance().isGamePieceInIntake();
    addRequirements(UpperShooter.getInstance(), LowerShooter.getInstance());
    intakeCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeCommand.end(interrupted);
    Intake.getInstance().setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished || intakeCommand.isFinished();
  }
}
