// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CreateButton {

  public CreateButton(Trigger button, Supplier<Command> automation,
    double elevatorPoseEnd) {
    button.whileTrue(automation.get()).whileFalse(
      new SetElevator(elevatorPoseEnd).alongWith(new InstantCommand(
        () -> Shooter.getInstance().setPower(0)
      ))
    );
  }

  public CreateButton(Trigger button, Command automation,
    double elevatorPoseEnd) {
    this(button, () -> automation, elevatorPoseEnd);
  }

  public CreateButton(Trigger button, Command automation) {
    this(button, automation, ElevatorConstants.defaultPose);
  }

  public CreateButton(Trigger button, Supplier<Command> automation) {
    this(button, automation, ElevatorConstants.defaultPose);
  }
}
