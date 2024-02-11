// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndRingCenter extends SequentialCommandGroup {
  /** Creates a new RunIntake. */
  public IntakeAndRingCenter(double power) {

    addCommands(
      new IntakeCommand(power),
      new SetElevator(ElevatorConstants.CENTER_POSE ),
      new CenterRing()
    );
  }
}
