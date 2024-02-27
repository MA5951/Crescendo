// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.Limelight;
import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.commands.MotorCommand;
import com.ma5951.utils.commands.RunInternallyControlledSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;
import frc.robot.automations.AMPScore;
import frc.robot.automations.AutoShoot;
import frc.robot.automations.CenterRing;
import frc.robot.automations.GoToAmp;
import frc.robot.automations.ResetAll;
import frc.robot.automations.RunIntake;
import frc.robot.automations.RunShoot;
import frc.robot.automations.ScoreWithoutAdjust;
import frc.robot.automations.SourceIntake;
import frc.robot.automations.Auto.FeedToShooter;
import frc.robot.automations.Auto.FourGamePieces;
import frc.robot.automations.Auto.TwoPieceCloseAmp;
import frc.robot.automations.Auto.TwoPieceCloseStage;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.elevator.ResetElevator;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.commands.swerve.DriveSwerveCommand;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED.INTAKE;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.UpperShooter;

public class RobotContainer {
  private final MAShuffleboard board = new MAShuffleboard("Autonamous");

  public enum IntakePose {
    FLOOR,
    SOURCE,
  }

  public static IntakePose intakepose = IntakePose.FLOOR;
  public static boolean isAmp = false;
  public static boolean ShootingLinkedToSpeaker = false;
  public static int factor = 1;

  public static final CommandPS5Controller
    driverController = new CommandPS5Controller(PortMap.Controllers.driveID);
  public static final CommandPS5Controller
    operatorController = new CommandPS5Controller(PortMap.Controllers.operatorID);

  public static final Limelight APRILTAGS_LIMELIGHT = new Limelight(
    "limelight-one", 0.25, 33.8);

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

    NamedCommands.registerCommand("Shoot linked to speaker side", 
    new RunInternallyControlledSubsystem(UpperShooter.getInstance(),
    () -> ShooterConstants.SPEAKER_UPPER_V_AUTO_SIDE, false)
    .alongWith(
      new RunInternallyControlledSubsystem(LowerShooter.getInstance(),
    () -> ShooterConstants.SPEAKER_LOWER_V_AUTO_SIDE, false))
      .andThen(new FeedToShooter())
    );

    NamedCommands.registerCommand(
      "stop shooter", new RunInternallyControlledSubsystem(UpperShooter.getInstance(),
      () -> 0d, true)
      .alongWith(
        new RunInternallyControlledSubsystem(LowerShooter.getInstance(),
      () -> 0d, true)));

    NamedCommands.registerCommand("ResetElevator",
      new ResetElevator()
    );

    NamedCommands.registerCommand("Center Ring", new CenterRing());

    NamedCommands.registerCommand("Feed To Shooter", new FeedToShooter());
    
    NamedCommands.registerCommand("Set Shooter Speed", new 
      RunInternallyControlledSubsystem(UpperShooter.getInstance(),
      () -> ShooterConstants.SPEAKER_UPPER_V_AUTO, false)
      .alongWith(
        new RunInternallyControlledSubsystem(LowerShooter.getInstance(),
      () -> ShooterConstants.SPEAKER_LOWER_V_AUTO, false))
    );

    NamedCommands.registerCommand("Set Shooter Speed side", new 
      RunInternallyControlledSubsystem(UpperShooter.getInstance(),
      () -> ShooterConstants.SPEAKER_UPPER_V_AUTO_SIDE, false)
      .alongWith(
        new RunInternallyControlledSubsystem(LowerShooter.getInstance(),
      () -> ShooterConstants.SPEAKER_LOWER_V_AUTO_SIDE, false))
    );

    NamedCommands.registerCommand("close intake", new RunInternallyControlledSubsystem(
      Elevator.getInstance(), () -> ElevatorConstants.SHOOTING_POSE, false)
    );
    
    NamedCommands.registerCommand("Update Limelight", new InstantCommand(
      () -> SwerveDrivetrainSubsystem.getInstance().addVisionMeasurement()
    ));
    
  }

  public RobotContainer() {
    registerCommands();

    SwerveDrivetrainSubsystem.getInstance();

    board.initSendableChooser("Autonomous Paths");

    board.addOptionToChooser("Four Game Pieces", new FourGamePieces()); // 4 game piece
    board.addOptionToChooser("Two Piece Stage", AutoBuilder.buildAuto("Two pice Stage")); // 2 Game Piece Stage
    board.addOptionToChooser("Two Piece Amp", AutoBuilder.buildAuto("Two pice Amp")); // 2 Game Piece Amp
    board.addOptionToChooser("Two Piece Middle", AutoBuilder.buildAuto("Two pice Middle")); // 2 Game Piece Middle
    board.addOptionToChooser("Three Piece Middle", AutoBuilder.buildAuto("Two pice Middle")
      .andThen(AutoBuilder.buildAuto("Theree pice Spaker middle"))); // 3 Game Piece Middle
    board.addOptionToChooser("Theree pice Amp", AutoBuilder.buildAuto("Theree pice Amp")); // 3 Game Piece Amp
    board.addOptionToChooser("Theree pice Stage", AutoBuilder.buildAuto("Theree pice Stage")); // 3 Game Piece Stage
    board.addOptionToChooser("2 note far stage", AutoBuilder.buildAuto("2 note far stage")); // 2 Game Piece Stage Far
    board.addOptionToChooser("one game piece", AutoBuilder.buildAuto("one game piece")); // One Game Piece
    board.addOptionToChooser("three Piece Close Amp", new TwoPieceCloseAmp()); // Two piece close amp
    board.addOptionToChooser("three Piece Close Stage", new TwoPieceCloseStage());// Two piece close amp
    board.addOptionToChooser("none", null); // none

    board.addDefaultOptionToChooser("none", null);

    configureBindings();
  }

  private void configureBindings() {
    driverController.R2().whileTrue(new InstantCommand(
      () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(
        0.4)
    )).whileFalse(
      new InstantCommand(
      () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(
        1)
    ));

    driverController.triangle().whileTrue(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::updateOffset)
    );

    driverController.touchpad().whileTrue(
      new DriveSwerveCommand(
        () -> -RobotContainer.driverController.getLeftX(),
        () -> -RobotContainer.driverController.getLeftY(),
        RobotContainer.driverController::getRightX,
        false)
    );

    // // // ---------------------------------------------------------------

    new CreateButton(driverController.povLeft(), new ResetElevator());

    driverController.povUp().whileTrue(new MotorCommand(
      Elevator.getInstance(), 0.3, 0
    )).whileFalse(
      new InstantCommand(() -> Elevator.getInstance().setSetPoint(Elevator.getInstance().getPosition())));
    
    driverController.povDown().whileTrue(new MotorCommand(
      Elevator.getInstance(), -0.3, 0
    )).whileFalse(
      new InstantCommand(() -> Elevator.getInstance().setSetPoint(Elevator.getInstance().getPosition())));

    // shooting linked to the speaker 
    new CreateButton(driverController.L1(), 
    new ScoreWithoutAdjust(
        () -> ShooterConstants.SPEAKER_UPPER_V, 
        () -> ShooterConstants.SPEAKER_LOWER_V,
          ElevatorConstants.SHOOTING_POSE)
          .alongWith(new InstantCommand(() -> isIntakeRunning = false))
    );

    // auto amp score
    new CreateButton(driverController.circle(),
     new GoToAmp().andThen(new AMPScore().alongWith(
        new InstantCommand(() -> isAmp = false)
      ))
    );
    
    // amp
    new CreateButton(driverController.button(9), new AMPScore().alongWith(
        new InstantCommand(() -> isAmp = false)
      ));

    
    // shooting normal
    new CreateButton(driverController.square(), new RunShoot());

    driverController.L2().whileTrue(new AutoShoot()).whileFalse(
      new ResetAll(() -> ElevatorConstants.DEFAULT_POSE).alongWith(new InstantCommand(() -> 
        SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(1)))
    );

    // floor or source intake
    driverController.R1().onTrue( 
      new ConditionalCommand(new RunIntake(IntakeConstants.INTAKE_POWER),
        new SourceIntake(),
        RobotContainer::IsFloor
      ).alongWith(new InstantCommand(() -> isIntakeRunning = true)).andThen(
        new ResetAll(() ->ElevatorConstants.DEFAULT_POSE)
      .alongWith(new InstantCommand(() -> isIntakeRunning = false)))
    );

    new CreateButton(driverController.povRight(), 
      new InstantCommand(() -> Elevator.getInstance().toggleDeafultPose()));

    // eject
    new CreateButton(driverController.cross(),
          new SetElevator(ElevatorConstants.SHOOTING_POSE).andThen(new MotorCommand(
          Intake.getInstance(), -IntakeConstants.INTAKE_POWER, 0)).alongWith(
          new InstantCommand(() -> isIntakeRunning = false))
        );

    // climb
    operatorController.triangle().whileTrue(
      new SetElevator(ElevatorConstants.CLIMB_POSE)
    );


    operatorController.cross().onTrue(
      new InstantCommand (() -> Elevator.getInstance().configMotorsForClimb()).andThen(
      new SetElevator(ElevatorConstants.CLOSE_CLIMB_POSE))
    .andThen(new InstantCommand (() -> Elevator.getInstance().configMotors())));


    // elevator change + amp or shoot chooser
    new CreateButton(operatorController.L1(),
      new SetElevator(ElevatorConstants.AMP_POSE).alongWith(
        new InstantCommand(() -> isAmp = true)
      ),
      () -> ElevatorConstants.AMP_POSE);

    new CreateButton(operatorController.povDown(), 
      new SetElevator(ElevatorConstants.DEFAULT_POSE),
      () -> ElevatorConstants.DEFAULT_POSE);

    // // choosing btween floor intake and source
    operatorController.povLeft().whileTrue(
      new InstantCommand(() -> intakepose = IntakePose.FLOOR)
      .alongWith(new InstantCommand(() -> LED.getInstance().setIntakeAnimation(INTAKE.GROUND)))
    );

    operatorController.povRight().whileTrue(
      new InstantCommand(() -> intakepose = IntakePose.SOURCE)
      .alongWith(new InstantCommand(() -> LED.getInstance().setIntakeAnimation(INTAKE.SOURCE)))
    );

    // choosing btween linked shoot or normal one
    operatorController.R2().whileTrue(
      new InstantCommand(() -> ShootingLinkedToSpeaker = true)
    );

    operatorController.L2().whileTrue(
      new InstantCommand(() -> ShootingLinkedToSpeaker = false)
    );

    // // //--------------------LEDS-----------------------

    operatorController.square().whileTrue(
      new InstantCommand(() -> LED.getInstance().activateAmp())
    ).whileFalse(new InstantCommand(() -> LED.getInstance().activateCOLAB()));

  }

  public Command getAutonomousCommand() {
    return board.getSelectedCommand();
  }
}
