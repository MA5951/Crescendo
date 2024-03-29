// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.AutoAutomations;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.AngleAdjust;
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
  private SwerveDrivetrainSubsystem swerve;

  private final double delay = 0.8;
  private final double tair = 0.22;

  private double intaionalPose = 0;
  private double deltaY = 0;

  public static boolean isRunning = false;

  public ShootInMotion() {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    swerveCommand = new AngleAdjust(() -> 
      {return DriverStation.getAlliance().get() == Alliance.Blue ? Math.PI : 0;},
      RobotContainer.driverController::getLeftX, () -> 0d);
    addRequirements(swerve,
      Intake.getInstance());
  }

    public void setShooter() {
    double[] shootingValue = ShooterConstants.sample(swerve.disFromSpeakerX);
    LowerShooter.getInstance().setSetPoint(
      shootingValue[1] * ShooterConstants.V_FACTOR);

    UpperShooter.getInstance().setSetPoint(
      shootingValue[0] * ShooterConstants.V_FACTOR);

  }

  private void calculateDeltaY() {
    deltaY = SwerveConstants.SPEAKER_TARGET_Y - swerve.getPose().getY();
  }

  private double getVelocityFactor() {
    calculateDeltaY();
    double v = (deltaY / (tair + delay));
    SwerveConstants.lowerSpeedFactor = Math.abs(v / SwerveConstants.MAX_VELOCITY);
    return Math.abs(v / SwerveConstants.MAX_VELOCITY);
  }

  public boolean canShoot() {
    calculateDeltaY();
    return Math.abs(swerve.getPose().getY() - (intaionalPose + deltaY)) <
      SwerveConstants.SHOOTING_IN_MOTION_TOLORANCE;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveCommand.initialize();
    intaionalPose = swerve.getPose().getY();
    setShooter();
    swerve.FactorVelocityTo(getVelocityFactor());
    isRunning = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveCommand.execute();
    
    if (canShoot() && UpperShooter.getInstance().atPoint()
        && LowerShooter.getInstance().atPoint()) {
      Intake.getInstance().setPower(IntakeConstants.INTAKE_POWER);
    } else {
      Intake.getInstance().setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveCommand.end(interrupted);
    swerve.FactorVelocityTo(1);
    Intake.getInstance().setPower(0);
    isRunning = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return swerve.disFromSpeakerX > SwerveConstants.MAX_SHOOT_DISTANCE || RobotContainer.isIntakeRunning;
  }
}
