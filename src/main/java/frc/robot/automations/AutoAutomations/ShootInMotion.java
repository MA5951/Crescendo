// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.AutoAutomations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class ShootInMotion extends Command {
  /** Creates a new ShootInMotion. */
  private SwerveDrivetrainSubsystem swerve;
  private Command feeCommand;
  private Double angle;
  private Boolean shoot;


  public ShootInMotion() {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    feeCommand = new MotorCommand(Intake.getInstance(),
     IntakeConstants.INTAKE_POWER, IntakeConstants.INTAKE_POWER);
    addRequirements(swerve);
  }

  //Swerve drive command that includes larger dead zone on Y and rotation lock when not spining
  public void swerveDrive() {
    double xSpeed = SwerveDrivetrainSubsystem.getInstance().isXYReversed ?
     RobotContainer.driverController.getLeftY() : RobotContainer.driverController.getLeftX();
    double ySpeed = SwerveDrivetrainSubsystem.getInstance().isXYReversed ?
     RobotContainer.driverController.getLeftX() : RobotContainer.driverController.getLeftY();
    
     double turningSpeed = RobotContainer.driverController.getRightX();

    xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : xSpeed;
    ySpeed = Math.abs(ySpeed) < 0.25 ? 0 : ySpeed;

    turningSpeed = (Math.abs(turningSpeed) < 0.25 ? 
    angle = DriverStation.getAlliance().get() == Alliance.Blue ?
    angle - Math.PI : angle 
    : turningSpeed) * -1;

    xSpeed = xSpeed *
        swerve.maxVelocity *
        (SwerveDrivetrainSubsystem.getInstance().isXReversed ? -1 : 1);
    ySpeed = ySpeed *
        swerve.maxVelocity *
        (SwerveDrivetrainSubsystem.getInstance().isYReversed ? -1 : 1);
    turningSpeed = turningSpeed *
        swerve.maxAngularVelocity;

    swerve.drive(xSpeed, ySpeed, turningSpeed, true);
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
        return swerve.getPose().getX() < SwerveConstants.MAX_SHOOTING_DISTANCE_BLUE || feeCommand.isFinished();
     } else {
       return swerve.getPose().getX() > SwerveConstants.MAX_SHOOTING_DISTANCE_RED || feeCommand.isFinished();
     }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.FactorVelocityTo(0.4); //Factor velocity to 2.6m/s
    feeCommand.initialize();
    setShooterAndElevator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive();
    setShooterAndElevator();


    if (Math.abs(swerve.getPose().getX() - SwerveConstants.SHOOTING_POSE_MOTION)
     <= SwerveConstants.SHOOTING_POSE_MOTION_TOLORANCE) {
      feeCommand.execute();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.FactorVelocityTo(1);
    Intake.getInstance().setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return false;
  }
}
