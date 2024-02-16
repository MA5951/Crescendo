// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;
import java.util.function.Supplier;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.swerve.AngleAdjust;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAutomation extends SequentialCommandGroup {

  public ScoreAutomation(Supplier<Double> up, Supplier<Double> down) {
    addCommands(
      new ParallelCommandGroup(
        new MotorCommand(Intake.getInstance(),
          IntakeConstants.INTAKE_POWER, 0).repeatedly(),
        new AngleAdjust(Shoot::getAngle, () -> 0d, () -> 0d).repeatedly(),
        new SetShooter(up, down).repeatedly()
      )
    );
  }
}
