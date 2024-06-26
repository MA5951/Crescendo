// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.RunLockModules;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreWithoutAdjust extends SequentialCommandGroup {

  public ScoreWithoutAdjust(Supplier<Double> upperVel,
    Supplier<Double> lowerVel, double elevatorPose) {
    addCommands(
      new GettingReadyToScore(upperVel, lowerVel, () -> elevatorPose),
      new ScoreAutomation(upperVel, lowerVel).alongWith(new RunLockModules())
    );
  }
}
