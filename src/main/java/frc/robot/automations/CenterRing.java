// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterRing extends SequentialCommandGroup {
  
  public CenterRing() {
    addCommands(
      new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new WaitUntilCommand(UpperShooter.getInstance()::isGamePiceInShooter),
            new WaitCommand(0.1)
          ),
          new InstantCommand(() -> Intake.getInstance().setPower(-0.8)),
          new MotorCommand(LowerShooter.getInstance(), 0.1, 0),
          new MotorCommand(UpperShooter.getInstance(), 0, 0)
        ),
        new ParallelDeadlineGroup(
          new AdjustRing(),
          new MotorCommand(LowerShooter.getInstance(), -0.4, 0),
          new MotorCommand(UpperShooter.getInstance(), -0.4, 0)
        ),
        new InstantCommand(() -> UpperShooter.getInstance().setSetPoint(0))
          .alongWith(
        new InstantCommand(() -> LowerShooter.getInstance().setSetPoint(0)))
    );
  }
}
