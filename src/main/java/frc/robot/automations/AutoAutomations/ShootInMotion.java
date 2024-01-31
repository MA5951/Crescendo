// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.AutoAutomations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.SwereDriveWhileShooting;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class ShootInMotion extends Command {
  /** Creates a new ShootInMotion. */
  private SwereDriveWhileShooting swerveCommand;
  private SwerveDrivetrainSubsystem swerve;
  private Command feeCommand;

  public ShootInMotion() {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    swerveCommand = new SwereDriveWhileShooting();
    feeCommand = new MotorCommand(Intake.getInstance(),
     IntakeConstants.INTAKE_POWER, IntakeConstants.INTAKE_POWER).repeatedly();
    addRequirements(SwerveDrivetrainSubsystem.getInstance());
  }

  

  //Function that sets the shooter and elevator based on the distance to the speaker
  public void setShooterAndElevator() {
    LowerShooter.getInstance().setSetPoint(
      LowerShooter.getInstance().getVelocityForShooting());

    UpperShooter.getInstance().setSetPoint(
      UpperShooter.getInstance().getVelocityForShooting());

    Elevator.getInstance().setSetPoint(
      Elevator.getInstance().getPoseForShoot());
  }

  //Returns if the robot is in a valid shooting range
  public boolean validShootingRange() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
        return swerve.getPose().getX() < SwerveConstants.MAX_SHOOTING_DISTANCE_BLUE;
     } else {
       return swerve.getPose().getX() > SwerveConstants.MAX_SHOOTING_DISTANCE_RED;
     }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.FactorVelocityTo(0.4); //Factor velocity to 2.6m/s
    swerveCommand.initialize();;
    feeCommand.initialize();
    setShooterAndElevator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveCommand.execute();
    setShooterAndElevator();


    if (Math.abs(swerve.getPose().getY() - SwerveConstants.SHOOTING_POSE_MOTION)
     <= SwerveConstants.SHOOTING_POSE_MOTION_TOLORANCE && validShootingRange()) {
      feeCommand.execute();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveCommand.end(interrupted);
    swerve.FactorVelocityTo(1);// Return velocity to 5.3m/s
    Intake.getInstance().setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return false ;//add ramp rate
  }
}
