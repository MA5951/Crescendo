// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.AutoAutomations;

import edu.wpi.first.math.geometry.Translation2d;
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
  public static boolean isRunning = false;
  public double disFromSpeaker;

  public ShootInMotion() {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    swerveCommand = new AngleAdjust(() -> 
      {return DriverStation.getAlliance().get() == Alliance.Blue ? Math.PI : 0;},
      () -> 0d, RobotContainer.driverController::getLeftY);
    addRequirements(SwerveDrivetrainSubsystem.getInstance(),
      Intake.getInstance());
  }

  

  //Function that sets the shooter and elevator based on the distance to the speaker
  public void setShooter() {
    disFromSpeaker = new Translation2d(
        SwerveDrivetrainSubsystem.getInstance().getPose().getX(),
         0).getDistance(new Translation2d(
          DriverStation.getAlliance().get() == Alliance.Blue ?
          SwerveConstants.SPEAKER_TARGET_X_BLUE : 
          SwerveConstants.SPEAKER_TAGET_X_RED, 0));
    double[] shootingValue = ShooterConstants.sample(disFromSpeaker);
    LowerShooter.getInstance().setSetPoint(
      shootingValue[1] * ShooterConstants.V_FACTOR);

    UpperShooter.getInstance().setSetPoint(
      shootingValue[0] * ShooterConstants.V_FACTOR);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.FactorVelocityTo(SwerveConstants.SHOOTING_SPEED);
    swerveCommand.initialize();
    setShooter();
    isRunning = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveCommand.execute();

    if (Math.abs(swerve.getPose().getY() - SwerveConstants.SHOOTING_POSE_MOTION)
     < (SwerveConstants.SHOOTING_POSE_MOTION_TOLORANCE)
        && UpperShooter.getInstance().atPoint()
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
     return false;
  }
}
