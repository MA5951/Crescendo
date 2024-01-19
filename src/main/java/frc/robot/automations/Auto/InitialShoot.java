// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.automations.GettingReadyToScore;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InitialShoot extends SequentialCommandGroup {
  /** Creates a new InitialShoot. */
  public InitialShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GettingReadyToScore(() ->ShooterConstants.speakerUpperV, () ->ShooterConstants.speakerLowerV, ElevatorConstants.shootingPoseAuto),
      new FeedToShooter()
    );
  }
}
