// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourceIntake extends SequentialCommandGroup {

  public SourceIntake() {
    addCommands(
      new SetElevator(ElevatorConstants.sourcePose),
      new ParallelDeadlineGroup(
        new IntakeCommand(-IntakeConstants.intakePower),
        new MotorCommand(UpperShooter.getInstance(), ShooterConstants.intakeSorceV, ShooterConstants.intakeSorceV),
        new MotorCommand(LowerShooter.getInstance(), ShooterConstants.intakeSorceV, ShooterConstants.intakeSorceV)
      ),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> {
          return !Intake.getInstance().isGamePieceInIntake();
        }),
        new MotorCommand(Intake.getInstance(), -IntakeConstants.intakePower, 0)
      ),
      new IntakeCommand(IntakeConstants.intakePower)
    );
  }
}
