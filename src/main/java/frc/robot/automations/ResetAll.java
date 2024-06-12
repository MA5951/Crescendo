// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetAll extends SequentialCommandGroup {

  public ResetAll(Supplier<Double> elevatorPose) {
    addCommands(
      new InstantCommand(
        () -> Elevator.getInstance().setSetPoint(elevatorPose.get()))
      .alongWith(
      new InstantCommand(() -> Intake.getInstance().setPower(0)).alongWith(
      new InstantCommand(
        () -> UpperShooter.getInstance().setSetPoint(ShooterConstants.defaultVUp))
        .alongWith(new InstantCommand(
        () -> LowerShooter.getInstance().setSetPoint(ShooterConstants.defaultVDown))))
      ).alongWith(new InstantCommand(
        () -> LowerShooter.getInstance().chengeIDLmode(IdleMode.kBrake)))
        .alongWith(
          new InstantCommand(
        () -> UpperShooter.getInstance().chengeIDLmode(IdleMode.kBrake))
      ).alongWith(
        new InstantCommand(() -> UpperShooter.getInstance().changeToDefaultV = true)
      ).alongWith(
        new InstantCommand(() -> UpperShooter.isShooting = false)
      )
    );
  }
}
