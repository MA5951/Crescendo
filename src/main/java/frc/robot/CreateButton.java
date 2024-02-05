// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.automations.ResetAll;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.UpperShooter;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CreateButton {

  public CreateButton(Trigger button, Command automation,
    double elevatorEndPose) {
    button.whileTrue(automation.alongWith(
      new InstantCommand(() -> UpperShooter.getInstance().changeToDefaultV = false)
    )).whileFalse(
      new ResetAll(elevatorEndPose).alongWith(
        new InstantCommand(() -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(1)))
    );
  }

  public CreateButton(Trigger button, Command automation) {
    this(button, automation, ElevatorConstants.DEFAULT_POSE);
  }
}
