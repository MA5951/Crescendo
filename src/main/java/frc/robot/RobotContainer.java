// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.Limelight;
import com.ma5951.utils.commands.MotorCommand;
import com.ma5951.utils.commands.RunInternallyControlledSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;
import frc.robot.automations.AMPScore;
import frc.robot.automations.AdjustRing;
import frc.robot.automations.CenterRing;
import frc.robot.automations.GettingReadyToScore;
import frc.robot.automations.IntakeAndRingCenter;
import frc.robot.automations.IntakeAutomation;
import frc.robot.automations.ResetAll;
import frc.robot.automations.RunIntake;
import frc.robot.automations.RunShoot;
import frc.robot.automations.ScoreWithoutAdjust;
import frc.robot.automations.Shoot;
import frc.robot.automations.ShootInMotion;
import frc.robot.automations.SourceIntake;
import frc.robot.automations.Auto.FeedToShooter;
import frc.robot.automations.Auto.FourGamePieces;
import frc.robot.automations.Auto.SetForAmp;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.elevator.ResetElevator;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.swerve.AngleAdjust;
import frc.robot.commands.swerve.DriveSwerveCommand;
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
  public static boolean isAmp = false;
  public static boolean ShootingLinkedToSpeaker = false;

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
    
    NamedCommands.registerCommand("Shoot linked to speaker",
      new RunInternallyControlledSubsystem(UpperShooter.getInstance(),
      () -> ShooterConstants.SPEAKER_UPPER_V_AUTO, false)
      .alongWith(
        new RunInternallyControlledSubsystem(LowerShooter.getInstance(),
      () -> ShooterConstants.SPEAKER_LOWER_V_AUTO, false))
        .andThen(new FeedToShooter())
    );

    NamedCommands.registerCommand(
      "stop shooter", new InstantCommand(() -> UpperShooter.getInstance().setSetPoint(0))
      .alongWith(new InstantCommand(() -> LowerShooter.getInstance().setSetPoint(0))));

    NamedCommands.registerCommand("ResetElevator",
      new ResetElevator()
    );

    NamedCommands.registerCommand("Center Ring", new CenterRing());

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
    // driverController.R2().whileTrue(new InstantCommand(
    //   () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(
    //     0.4)
    // )).whileFalse(
    //   new InstantCommand(
    //   () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(
    //     1)
    // ));

    // driverController.triangle().whileTrue(
    //   new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::updateOffset)
    // );

    // driverController.button(9).whileTrue(
    //   new AngleAdjust(
    //     () -> Math.toRadians(90),
    //     RobotContainer.driverController::getLeftX,
    //     RobotContainer.driverController::getLeftY)
    // );

    // driverController.touchpad().whileTrue(
    //   new DriveSwerveCommand(
    //     () -> -RobotContainer.driverController.getLeftX(),
    //     () -> -RobotContainer.driverController.getLeftY(),
    //     RobotContainer.driverController::getRightX,
    //     false)
    // );

    // // // ---------------------------------------------------------------

    new CreateButton(driverController.povLeft(), new ResetElevator());

    // driverController.povUp().whileTrue(new MotorCommand(
    //   Elevator.getInstance(), 0.3, 0
    // ))//.whileFalse(
    //   //new InstantCommand(() -> Elevator.getInstance().setSetPoint(Elevator.getInstance().getPosition()))
    // ;
    
    // driverController.povDown().whileTrue(new MotorCommand(
    //   Elevator.getInstance(), -0.3, 0
    // ))//.whileFalse(
    //   //new InstantCommand(() -> Elevator.getInstance().setSetPoint(Elevator.getInstance().getPosition()))
    // ;

    // shooting linked to the speaker 
    new CreateButton(driverController.L1(), 
    new ScoreWithoutAdjust(
        () -> ShooterConstants.SPEAKER_UPPER_V, 
        () -> ShooterConstants.SPEAKER_LOWER_V,
          ElevatorConstants.SHOOTING_POSE)
          .alongWith(new InstantCommand(() -> isIntakeRunning = false))
    );
    
    // amp
    new CreateButton(driverController.circle(), new AMPScore().alongWith(
        new InstantCommand(() -> isAmp = false)
      ));

    
    // shooting normal
    new CreateButton(driverController.square(), new RunShoot());

    // shooting in motion
    // new CreateButton(
    //   new Trigger(
    //     () -> {return driverController.getL2Axis() > 0.1
    //       && (SwerveDrivetrainSubsystem.getInstance().disFromSpeakerX
    //        < SwerveConstants.MAX_SHOOT_DISTANCE && !isIntakeRunning);}
    //   ),new ShootInMotion());

    //auto align
    // new CreateButton(
    //   new Trigger(
    //     () -> driverController.L2().getAsBoolean()
    //     && !SwerveDrivetrainSubsystem.getInstance().canShoot()),
    //     new AngleAdjust(Shoot::getAngle, RobotContainer.driverController::getLeftX,
    //     RobotContainer.driverController::getLeftY)
    //   );

    // new CreateButton(
    //   new Trigger(
    //     () -> driverController.L2().getAsBoolean() &&
    //     SwerveDrivetrainSubsystem.getInstance().canShoot()
    //   ), new RunShoot()
    // );

    // floor or source intake
    driverController.R1().onTrue( 
      new ConditionalCommand(new RunIntake(IntakeConstants.INTAKE_POWER),
        new SourceIntake(),
        RobotContainer::IsFloor
      ).alongWith(new InstantCommand(() -> isIntakeRunning = true)).andThen(
        new ResetAll(ElevatorConstants.DEFAULT_POSE)
      .alongWith(new InstantCommand(() -> isIntakeRunning = false)))
    );

    // // eject
    // new CreateButton(driverController.cross(), new MotorCommand(
    //   Intake.getInstance(), -IntakeConstants.INTAKE_POWER, 0).alongWith(
    //     new InstantCommand(() -> isIntakeRunning = false).alongWith(
    //       new SetElevator(ElevatorConstants.SHOOTING_POSE)
    //     )));

    // // climb
    // new CreateButton(operatorController.triangle(), 
    //   new SetElevator(ElevatorConstants.CLIMB_POSE),
    //   ElevatorConstants.CLIMB_POSE);
  
    // new CreateButton(operatorController.cross(), 
    //   new SetElevator(ElevatorConstants.CLOSE_CLIMB_POSE),
    //   ElevatorConstants.CLOSE_CLIMB_POSE);

    // // elevator change + amp or shoot chooser
    // new CreateButton(operatorController.L1(), 
    //   new SetElevator(ElevatorConstants.AMP_POSE).alongWith(
    //     new InstantCommand(() -> isAmp = true)
    //   ),
    //   ElevatorConstants.AMP_POSE);

    // new CreateButton(operatorController.povDown(), 
    //   new SetElevator(ElevatorConstants.DEFAULT_POSE),
    //   ElevatorConstants.DEFAULT_POSE);

    // // choosing btween floor intake and source
    // operatorController.povLeft().whileTrue(
    //   new InstantCommand(() -> intakepose = IntakePose.FLOOR)
    // );

    // operatorController.povRight().whileTrue(
    //   new InstantCommand(() -> intakepose = IntakePose.SOURCE)
    // );

    // // choosing btween linked shoot or normal one
    // operatorController.R2().whileTrue(
    //   new InstantCommand(() -> ShootingLinkedToSpeaker = true)
    // );

    // operatorController.L2().whileTrue(
    //   new InstantCommand(() -> ShootingLinkedToSpeaker = false)
    // );

    // // //--------------------LEDS-----------------------

    // // operatorController.R1().whileTrue(
    // //   new InstantCommand(() -> LED.getInstance().activateAmp())
    // // );

    // // operatorController.L1().whileTrue(
    // //   new InstantCommand(() -> LED.getInstance().activateCoalition())
    // // );
  }
  public Command getAutonomousCommand() {
    // return new FourGamePieces(); // 4 gmae piece
    // return AutoBuilder.buildAuto("Two pice Stage"); // two pieces from stange side
    // return AutoBuilder.buildAuto("Two pice Middle")
    //   .andThen(AutoBuilder.buildAuto("Theree pice Spaker middle"));
    // return AutoBuilder.buildAuto("one game piece"); // one game piece
    // return AutoBuilder.buildAuto("Two pice Stage");
    // return AutoBuilder.buildAuto("Two pice Amp");
    // return AutoBuilder.buildAuto("Theree pice Amp");
    return AutoBuilder.buildAuto("Theree pice Stage");
  }
}
