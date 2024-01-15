// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import java.util.function.Supplier;

import com.ma5951.utils.commands.RunInternallyControlledSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GettingReadyToScore extends SequentialCommandGroup {

  public GettingReadyToScore(Supplier<Double> vel, double elevatorPose) {
    addCommands(
      new ParallelCommandGroup(
        new SetElevator(elevatorPose),
        new RunInternallyControlledSubsystem(
          Shooter.getInstance(), vel,
          false)
      )
    );
  }
}
