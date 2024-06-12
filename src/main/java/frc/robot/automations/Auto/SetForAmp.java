// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetForAmp extends SequentialCommandGroup {
  /** Creates a new AmpScore. */
  public SetForAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> Elevator.getInstance().setSetPoint(ElevatorConstants.AMP_POSE)),
      new InstantCommand(() -> UpperShooter.getInstance().setSetPoint(ShooterConstants.AMP_V_UPPER)),
      new InstantCommand(() -> UpperShooter.getInstance().setSetPoint(ShooterConstants.AMP_V_LOWER))
    );
  }
}
