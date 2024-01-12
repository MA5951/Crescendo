// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class SpeakerAngleAdjust extends Command {
  /** Creates a new SpeakerAngleAdjust. */
  private SwerveDrivetrainSubsystem swerve;
  private PIDController pid;
  private double xDis;
  private double  yDis;

  public SpeakerAngleAdjust() {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    addRequirements(swerve);

    pid = new PIDController(
      SwerveConstants.THATA_KP,
      SwerveConstants.THATA_KI,
      SwerveConstants.THATA_KD
    );

    pid.setTolerance(SwerveConstants.anglePIDTolorance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double xTrget = DriverStation.getAlliance().get() == Alliance.Red ? 
      SwerveConstants.speakerTargetXRed : SwerveConstants.speakerTargetXBlue;
    double yTrget = DriverStation.getAlliance().get() == Alliance.Red ? 
      SwerveConstants.speakerTargetYRed : SwerveConstants.speakerTargetYBlue;
    xDis = Math.abs(swerve.getPose().getX() - xTrget);
    yDis = Math.abs(swerve.getPose().getY() - yTrget);
    double angle = Math.atan(yDis / xDis);
    angle = DriverStation.getAlliance().get() == Alliance.Blue ?
      angle - Math.PI : angle;
    if (swerve.getPose().getY() > yTrget) {
      angle = -angle;
    }

    pid.setSetpoint(angle); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(
      0, 0,
      pid.calculate(swerve.getPose().getRotation().getRadians())
      , false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xDis > SwerveConstants.maxSpeakerDistanceX || 
            yDis > SwerveConstants.maxSpeakerDistanceY ||
              pid.atSetpoint();
  }
}
