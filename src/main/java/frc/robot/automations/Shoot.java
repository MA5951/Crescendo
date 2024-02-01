// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.AngleAdjust;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  private static SwerveDrivetrainSubsystem swerve =
    SwerveDrivetrainSubsystem.getInstance();
  public static double yDis;
  public static double xDis;

  private static double getAngle() {
    double xTrget = DriverStation.getAlliance().get() == Alliance.Red ? 
      SwerveConstants.SPEAKER_TAGET_X_RED : SwerveConstants.SPEAKER_TARGET_X_BLUE;
    double yTrget = DriverStation.getAlliance().get() == Alliance.Red ? 
      SwerveConstants.SPEAKER_TARGET_Y_RED : SwerveConstants.SPEAKER_TARGET_Y_BLUE;
    xDis = Math.abs(swerve.getPose().getX() - xTrget);
    yDis = Math.abs(swerve.getPose().getY() - yTrget);
    double angle = Math.atan(yDis / xDis);
    angle = DriverStation.getAlliance().get() == Alliance.Blue ?
      angle - Math.PI : angle;
    if (swerve.getPose().getY() > yTrget) {
      angle = -angle;
    }
    return -(angle);
  }

  public Shoot() {
    Supplier<Double> elevatorPose = ()  -> ElevatorConstants.DEFAULT_POSE;
    addCommands(
      new ParallelCommandGroup(
        new AngleAdjust(Shoot::getAngle, () -> 0d, () -> 0d),
        new GettingReadyToScore(
          UpperShooter.getInstance()::getVelocity,
          LowerShooter.getInstance()::getVelocity,
          elevatorPose)
      ),
      new ScoreAutomation()
    );
  }
}
