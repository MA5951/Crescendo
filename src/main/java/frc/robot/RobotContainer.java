// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;
import frc.robot.automations.AMPSpeaker;
import frc.robot.automations.RunShoot;
import frc.robot.automations.ScoreWithoutAdjust;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RobotContainer {
  private enum ScoringOptions {
    AMP,
    SPEAKER
  }

  public static ScoringOptions scoringOption = ScoringOptions.SPEAKER;

  public static final CommandPS5Controller
    driverController = new CommandPS5Controller(PortMap.Controllers.driveID);
  public static final CommandPS5Controller
    operatorController = new CommandPS5Controller(PortMap.Controllers.operatorID);
  
  private static Command GetScoreAutomation() {
    return scoringOption == ScoringOptions.SPEAKER ?
      new RunShoot(false) :
      new ScoreWithoutAdjust(() -> ShooterConstants.AMPV, ElevatorConstants.AMPPose);
  }

  private void registerCommands() {
  }

  public RobotContainer() {
    registerCommands();
    configureBindings();
  }

  private void configureBindings() {
    driverController.R2().whileTrue(new InstantCommand(
      () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(0.4)
    )).whileFalse(
      new InstantCommand(
      () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(1)
    ));

    driverController.triangle().whileTrue(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::updateOffset)
    );

    new CreateButton(driverController.R1(), new IntakeCommand());

    new CreateButton(driverController.L2(), new ScoreWithoutAdjust(
      () -> ShooterConstants.speakerV, ElevatorConstants.shootingPose));

    new CreateButton(driverController.L1(), new RunShoot(true));
    
    new CreateButton(driverController.circle(), new AMPSpeaker(RobotContainer::GetScoreAutomation));

    new CreateButton(operatorController.triangle(),
      new SetElevator(ElevatorConstants.climbPose),
      ElevatorConstants.closeClimbPose);
    
    operatorController.povUp().whileTrue(
      new InstantCommand(() -> scoringOption = ScoringOptions.SPEAKER)
    );

    operatorController.povUp().whileTrue(
      new InstantCommand(() -> scoringOption = ScoringOptions.AMP)
    );

    operatorController.touchpad().whileTrue(
      new InstantCommand(() -> LED.getInstance().activateAmp())
    );

    operatorController.options().whileTrue(
      new InstantCommand(() -> LED.getInstance().activateCoalition())
    );
  }
  public Command getAutonomousCommand() {
    return null;
  }
}
