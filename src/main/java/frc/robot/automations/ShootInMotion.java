// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.AngleAdjust;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class ShootInMotion extends Command {
  /** Creates a new ShootInMotion. */
  private AngleAdjust swerveCommand;
  private static SwerveDrivetrainSubsystem swerve;

  private static final double delay = 0.31;

  private boolean factoredV;

  private static double getAngle() {
    if (swerve.update) {
      return Shoot.getAngle();
    }
    return Math.toRadians(SwerveDrivetrainSubsystem.getInstance().getOffsetAngle() - 180);
  }

  public ShootInMotion() {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    swerveCommand = new AngleAdjust(ShootInMotion::getAngle,
      RobotContainer.driverController::getLeftX,
      RobotContainer.driverController::getLeftY,
      true, true);
    addRequirements(swerve, Intake.getInstance());
  }

  private void setShooter() {
    double[] shootingValue = ShooterConstants.sample(
      Math.sqrt(Math.pow(swerve.disFormSpeaker
    * swerve.getRotation2d().getSin() + delay * 
      swerve.getRobotRelativeSpeeds().vyMetersPerSecond, 2) + Math.pow(swerve.disFromSpeakerX
        + delay * 
      swerve.getRobotRelativeSpeeds().vxMetersPerSecond, 2)), ShooterConstants.shootingPoses
    );
    LowerShooter.getInstance().setSetPoint(
      shootingValue[1] * ShooterConstants.V_FACTOR);

    UpperShooter.getInstance().setSetPoint(
      shootingValue[0] * ShooterConstants.V_FACTOR);

  }

  private static double getTair() {
    return 0.2225 * swerve.disFromSpeakerX - 0.0544;
  }

  private static double getDeltaY() {
    return -(swerve.getRobotRelativeSpeeds().vyMetersPerSecond * (getTair() + delay));
  }

  private static double getVelocityFactor() {
    double deltaY1 = SwerveConstants.SPEAKER_TARGET_Y - swerve.getPose().getY();
    double v = (deltaY1 / (getTair() + delay));
    return Math.min(Math.abs(v / SwerveConstants.MAX_VELOCITY), 1) * 0.6;
  }

  private boolean canShoot() {
    return Math.abs(swerve.getPose().getY() + getDeltaY() * swerve.getPose().getRotation().getSin()
     - SwerveConstants.SPEAKER_TARGET_Y) <
      SwerveConstants.SHOOTING_IN_MOTION_TOLORANCE;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    UpperShooter.isShooting = true;
    swerveCommand.initialize();
    setShooter();
    swerve.update = false;
    factoredV = false;
    swerve.FactorVelocityTo(0.5);
    swerveCommand.setUseGyro(true);
    Elevator.getInstance().setSetPoint(ElevatorConstants.SHOOTING_POSE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (swerve.canShoot() && swerve.update) {
      swerve.FactorVelocityTo(0.14);
    }
    if (swerve.update && !factoredV) {
      swerveCommand.setUseGyro(false);
      factoredV = true;
    }
    setShooter();
    swerveCommand.execute();
    if (canShoot() && UpperShooter.getInstance().atPoint()
        && LowerShooter.getInstance().atPoint() && Elevator.getInstance().atPoint()
        && swerve.update && swerve.canShoot() && AngleAdjust.atPoint()) {
      Intake.getInstance().setPower(IntakeConstants.INTAKE_POWER);
    }
    System.out.println(Math.toDegrees(getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveCommand.end(interrupted);
    Intake.getInstance().setPower(0);
    swerve.FactorVelocityTo(1);
    UpperShooter.isShooting = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return RobotContainer.isIntakeRunning;
  }
}
