// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import java.util.function.Supplier;

import com.ma5951.utils.subsystem.InternallyControlledSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RunInternallyControlledSubsystem extends Command {
  /** Creates a new ControlCommand. */
  private InternallyControlledSubsystem subsystem;
  private Supplier<Double> setPoint;
  private boolean StopAtEnd;

  public RunInternallyControlledSubsystem(
    InternallyControlledSubsystem subsystem,
    Supplier<Double> setPoint,
    boolean StopAtEnd) {
    this.subsystem = subsystem;
    this.setPoint = setPoint;
    this.StopAtEnd = StopAtEnd;
    addRequirements(subsystem);
  }

  public RunInternallyControlledSubsystem(
    InternallyControlledSubsystem subsystem,
    double setPoint, boolean needToStopGivingPowerAtTheEnd) {
      this(subsystem, () -> setPoint, needToStopGivingPowerAtTheEnd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.setSetPoint(setPoint.get());
    subsystem.calculate(setPoint.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (StopAtEnd) {
      subsystem.setPower(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !subsystem.canMove() || subsystem.atPoint();
  }
}
