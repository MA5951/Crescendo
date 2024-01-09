// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommand extends Command {
  private Intake intake;
  private MotorCommand motorCommand;

  public IntakeCommand() {
    intake = Intake.getInstance();
    motorCommand = new MotorCommand(intake, IntakeConstants.intakePower, 0);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motorCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorCommand.end(interrupted);

    if (!interrupted) {
      intake.setPiceIn(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getSensor() || intake.isPiceIn();
  }
}
