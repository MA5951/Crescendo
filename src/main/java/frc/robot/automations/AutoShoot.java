// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.AngleAdjust;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  public AutoShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(0.3))
          .alongWith(new InstantCommand(
            () -> SwerveDrivetrainSubsystem.getInstance().update = false
          )).
          andThen(
          new ParallelDeadlineGroup(
            new WaitUntilCommand(
              () -> {
                return SwerveDrivetrainSubsystem.getInstance().update;
              }
            ), 
            new AngleAdjust(() -> Math.toRadians(
              SwerveDrivetrainSubsystem.getInstance().getOffsetAngle() - 180),
              RobotContainer.driverController::getLeftX,
            RobotContainer.driverController::getLeftY, true, true))
          ).
          andThen(
            new ParallelDeadlineGroup (
                new WaitUntilCommand(() -> {
                  return SwerveDrivetrainSubsystem.getInstance().disFormSpeaker < 
                  SwerveConstants.MAX_SHOOT_DISTANCE * 0.9;}),
                new AngleAdjust(Shoot::getAngle, RobotContainer.driverController::getLeftX,
                    RobotContainer.driverController::getLeftY, false, true)),
            // new InstantCommand(() -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(0.15)),
            new RunShoot().repeatedly())
    );
  }
}
