// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class AMPScore extends SequentialCommandGroup {

  public AMPScore() {
    addCommands(
      new SetElevator(ElevatorConstants.AMP_POSE),
      new ParallelCommandGroup(
      new InstantCommand(() -> 
          Intake.getInstance().setPower(IntakeConstants.INTAKE_POWER)),
        new MotorCommand(UpperShooter.getInstance(),
          ShooterConstants.AMP_V_UPPER, 0),
        new MotorCommand(LowerShooter.getInstance(),
          ShooterConstants.AMP_V_LOWER, 0)
      )
    );
  }
}
