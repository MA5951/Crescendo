// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.Limelight;
import com.ma5951.utils.commands.MotorCommand;
import com.ma5951.utils.commands.RunInternallyControlledSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

// import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;
import frc.robot.automations.AMPScore;
import frc.robot.automations.CenterRing;
import frc.robot.automations.GettingReadyToScore;
import frc.robot.automations.IntakeAndRingCenter;
import frc.robot.automations.IntakeAutomation;
import frc.robot.automations.ResetAll;
import frc.robot.automations.RunIntake;
import frc.robot.automations.RunShoot;
import frc.robot.automations.ScoreWithoutAdjust;
import frc.robot.automations.Shoot;
import frc.robot.automations.SourceIntake;
import frc.robot.automations.Auto.FeedToShooter;
import frc.robot.automations.Auto.SetForAmp;
import frc.robot.automations.AutoAutomations.ShootInMotion;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.elevator.ResetElevator;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;

public class RobotContainer {
  public enum IntakePose {
    FLOOR,
    SOURCE,
  }

  public static IntakePose intakepose = IntakePose.FLOOR;

  public static final CommandPS5Controller
    driverController = new CommandPS5Controller(PortMap.Controllers.driveID);
  public static final CommandPS5Controller
    operatorController = new CommandPS5Controller(PortMap.Controllers.operatorID);

  public static final Limelight APRILTAGS_LIMELIGHT = new Limelight(
    "limelight-one", new Transform3d());

  public static boolean isIntakeRunning = false;

  public static boolean IsFloor() {
    return intakepose == IntakePose.FLOOR;
  }

  private void registerCommands() {
    NamedCommands.registerCommand("Intake", new IntakeCommand(IntakeConstants.INTAKE_POWER));

    //   .andThen(new FeedToShooter()));
    NamedCommands.registerCommand("Shoot linked to speaker",
      new RunInternallyControlledSubsystem(UpperShooter.getInstance(),
      () -> ShooterConstants.SPEAKER_UPPER_V, false)
      .alongWith(
        new RunInternallyControlledSubsystem(LowerShooter.getInstance(),
      () -> ShooterConstants.SPEAKER_LOWER_V, false))
        .andThen(new FeedToShooter())
    );

    NamedCommands.registerCommand(
      "stop shooter", new InstantCommand(() -> UpperShooter.getInstance().setPower(0))
      .alongWith(new InstantCommand(() -> LowerShooter.getInstance().setPower(0))));

    NamedCommands.registerCommand("ResetElevator",
      new ResetElevator()
    );

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
    NamedCommands.registerCommand("Elvator Intake", new IntakeAutomation(IntakeConstants.INTAKE_POWER));
    
  }

  public RobotContainer() {
    registerCommands();
    configureBindings();
  }

  private void configureBindings() {
    driverController.R2().whileTrue(new InstantCommand(
      () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(
        SwerveConstants.lowerSpeedFactor)
    )).whileFalse(
      new InstantCommand(
      () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(
        SwerveConstants.lowerSpeedFactor / SwerveConstants.LOWER_SPEED)
    ));

    driverController.triangle().whileTrue(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::updateOffset)
    );

    // // ---------------------------------------------------------------

    new CreateButton(driverController.povLeft(), new ResetElevator());

    driverController.povUp().whileTrue(new MotorCommand(
      Elevator.getInstance(), 0.15, 0
    )).whileFalse(
      new InstantCommand(() -> Elevator.getInstance().setSetPoint(Elevator.getInstance().getPosition()))
    );
    
    driverController.povDown().whileTrue(new MotorCommand(
      Elevator.getInstance(), -0.15, 0
    )).whileFalse(
      new InstantCommand(() -> Elevator.getInstance().setSetPoint(Elevator.getInstance().getPosition()))
    );

    // shooting linked to the speaker 
    new CreateButton(driverController.L1(), new ScoreWithoutAdjust(
      () -> ShooterConstants.SPEAKER_UPPER_V, 
      () -> ShooterConstants.SPEAKER_LOWER_V,
        ElevatorConstants.DEFAULT_POSE));
    
    // shooting or amp
    new CreateButton(driverController.circle(), new AMPScore());

    new CreateButton(driverController.square(), new RunShoot());

    new CreateButton(
      new Trigger(
        () -> {return driverController.getL2Axis() > 0.5;}
      ), new ShootInMotion());

    // floor or source
    driverController.R1().onTrue( 
      new ConditionalCommand(new IntakeAutomation(IntakeConstants.INTAKE_POWER),
        new SourceIntake(),
        RobotContainer::IsFloor
      ).alongWith(new InstantCommand(() -> isIntakeRunning = true)).andThen(
        new ResetAll(ElevatorConstants.DEFAULT_POSE)
      ).alongWith(new InstantCommand(() -> isIntakeRunning = false))
    );

    // // climb
    // new CreateButton(operatorController.povUp(), 
    //   new SetElevator(ElevatorConstants.CLIMB_POSE),
    //   ElevatorConstants.CLIMB_POSE);
  
    // new CreateButton(driverController.povDown(), 
    //   new SetElevator(ElevatorConstants.CLOSE_CLIMB_POSE),
    //   ElevatorConstants.CLOSE_CLIMB_POSE);

    // // source intake
    // new CreateButton(operatorController.square(), new SourceIntake());

    // ring Center
    // new CreateButton(operatorController.povRight(), new CenterRing());
    
    // eject
    new CreateButton(driverController.cross(), new MotorCommand(
      Intake.getInstance(), -IntakeConstants.INTAKE_POWER, 0));

    // // choosing btween floor intake and sou
    // operatorController.povDown().whileTrue(
    //   new InstantCommand(() -> intakepose = IntakePose.FLOOR)
    // );

    // operatorController.povUp().whileTrue(
    //   new InstantCommand(() -> intakepose = IntakePose.SOURCE)
    // );

    // //--------------------LEDS-----------------------

    // operatorController.touchpad().whileTrue(
    //   new InstantCommand(() -> LED.getInstance().activateAmp())
    // );

    // operatorController.options().whileTrue(
    //   new InstantCommand(() -> LED.getInstance().activateCoalition())
    // );
  }
  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto("Two pice Middle");
  }
}
