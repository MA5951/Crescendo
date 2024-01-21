// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

// import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;
import frc.robot.automations.GettingReadyToScore;
import frc.robot.automations.IntakeAutomation;
import frc.robot.automations.ScoreWithoutAdjust;
import frc.robot.automations.Shoot;
import frc.robot.automations.Auto.FeedToShooter;
import frc.robot.automations.Auto.SetForAmp;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;

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
  
  private static boolean IsSpeaker() {
    return scoringOption == ScoringOptions.SPEAKER;
  }

  private void registerCommands() {
    NamedCommands.registerCommand("Intake", new IntakeCommand(IntakeConstants.intakePower));
    NamedCommands.registerCommand("Initial Shoot", new 
      GettingReadyToScore(() ->ShooterConstants.speakerUpperV, () ->ShooterConstants.speakerLowerV, ElevatorConstants.shootingPoseAuto)
      .andThen(new FeedToShooter()));

    NamedCommands.registerCommand("Feed To Shooter", new FeedToShooter());
    NamedCommands.registerCommand("Set Shooter Speed", new 
      SetShooter(
        UpperShooter.getInstance()::getVelocityForShooting,
        LowerShooter.getInstance()::getVelocityForShooting
      ));

    NamedCommands.registerCommand("Shoot", new 
      SetShooter(
        UpperShooter.getInstance()::getVelocityForShooting,
        LowerShooter.getInstance()::getVelocityForShooting
      ).andThen(new FeedToShooter()));

    NamedCommands.registerCommand("SetForAmp", new SetForAmp());
    NamedCommands.registerCommand("Elvator Intake", new IntakeAutomation(IntakeConstants.intakePower));
    
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

    // ---------------------------------------------------------------

    new CreateButton(driverController.R1(), 
      new IntakeCommand(IntakeConstants.intakePower));

    // shooting linked to the speaker 
    new CreateButton(driverController.L2(), new ScoreWithoutAdjust(
      () -> ShooterConstants.speakerUpperV, () -> ShooterConstants.speakerLowerV,
        ElevatorConstants.shootingPose));

    // shootiong linked to the podduim 
    new CreateButton(driverController.L1(), new Shoot(true));
    
    // shooting or amp
    new CreateButton(driverController.circle(), 
      new ConditionalCommand(new Shoot(false),
        new ScoreWithoutAdjust(() -> ShooterConstants.AMPV,
          () -> ShooterConstants.AMPV, ElevatorConstants.AMPPose), RobotContainer::IsSpeaker
        )
    );

    // climb
    new CreateButton(operatorController.triangle(),
      new SetElevator(ElevatorConstants.climbPose),
      ElevatorConstants.closeClimbPose);
    
    // eject
    new CreateButton(operatorController.cross(), new ScoreWithoutAdjust(
      () -> ShooterConstants.ejectV, () -> ShooterConstants.ejectV, ElevatorConstants.ejectPose
    ));
    
    // choosing btween apm score and amp score
    operatorController.povUp().whileTrue(
      new InstantCommand(() -> scoringOption = ScoringOptions.SPEAKER)
    );

    operatorController.povDown().whileTrue(
      new InstantCommand(() -> scoringOption = ScoringOptions.AMP)
    );

    // --------------------LEDS-----------------------

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
