// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class AMPSpeaker extends Command {
  private Supplier<Command> SupplierCommand;
  private Command command;

  public AMPSpeaker(Supplier<Command> SupplierCommand) {
    this.SupplierCommand = SupplierCommand;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command = SupplierCommand.get();
    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}