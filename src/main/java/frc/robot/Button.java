// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.automations.ResetAll;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.UpperShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Button {

  public static void Create(Trigger button, Command automation,
    Supplier<Double> elevatorEndPose) {
    button.whileTrue(automation.alongWith(
      new InstantCommand(() -> UpperShooter.getInstance().changeToDefaultV = false)
    )).whileFalse(
      new ResetAll(elevatorEndPose)
    );
  }

  public static void Create(Trigger button, Command automation) {
    Create(button, automation, () -> ElevatorConstants.DEFAULT_POSE);
  }
}
