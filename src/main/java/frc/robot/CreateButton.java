// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CreateButton {

  public CreateButton(Trigger button, Command automation,
    double elevatorEndPose) {
    button.whileTrue(automation).whileFalse(
      new InstantCommand(
        () -> Elevator.getInstance().setSetPoint(elevatorEndPose))
          .alongWith(
      new InstantCommand(() -> Intake.getInstance().setPower(0)).alongWith(
      new InstantCommand(() -> 
        RobotContainer.driverController.getHID()
          .setRumble(RumbleType.kLeftRumble, 0)
      ).alongWith(
      new InstantCommand(
        () -> UpperShooter.getInstance().setSetPoint(ShooterConstants.defaultV))
        .alongWith(new InstantCommand(
        () -> LowerShooter.getInstance().setSetPoint(ShooterConstants.defaultV)))))
      ).alongWith(new InstantCommand(
        () -> LowerShooter.getInstance().chengeIDLmode(IdleMode.kBrake)))
        .alongWith(
          new InstantCommand(
        () -> UpperShooter.getInstance().chengeIDLmode(IdleMode.kBrake))
      ).alongWith(
        new InstantCommand(() -> 
          RobotContainer.driverController.getHID()
          .setRumble(RumbleType.kBothRumble, 0))
      )
    );
  }

  public CreateButton(Trigger button, Command automation) {
    this(button, automation, ElevatorConstants.DEFAULT_POSE);
  }
}
