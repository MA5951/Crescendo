// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndRingCenter extends SequentialCommandGroup {
  /** Creates a new RunIntake. */
  public IntakeAndRingCenter(double power) {

    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new IntakeCommand(power),
          new ParallelDeadlineGroup(
            new WaitCommand(0.8),
            new InstantCommand(() -> Intake.getInstance().setPower(power))
          ),
          new AdjustRing()
        ),
      new MotorCommand(UpperShooter.getInstance(), 0, 0),
      new MotorCommand(LowerShooter.getInstance(), 0, 0))
    );
  }
}
