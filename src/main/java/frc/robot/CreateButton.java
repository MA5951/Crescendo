// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CreateButton {

  public CreateButton(Trigger button, Command automation,
    double elevatorPoseEnd) {
    button.whileTrue(automation).whileFalse(
      new InstantCommand(
        () -> Elevator.getInstance().setSetPoint(elevatorPoseEnd))
          .alongWith(
      new ConditionalCommand(
       new SetShooter(() -> 1000d, () -> 1000d), new InstantCommand(), 
       () -> {
        return UpperShooter.getInstance().getSetPoint() != 0;
       })
      .andThen(
      new InstantCommand(
        () -> UpperShooter.getInstance().setSetPoint(0))//)
        .alongWith(new InstantCommand(
        () -> LowerShooter.getInstance().setSetPoint(0))))
      )
    );
  }

  public CreateButton(Trigger button, Command automation) {
    this(button, automation, ElevatorConstants.defaultPose);
  }
}
