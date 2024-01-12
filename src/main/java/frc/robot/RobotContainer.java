// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.commands.MotorCommand;

//import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class RobotContainer {

  public static final CommandPS5Controller
    driverController = new CommandPS5Controller(PortMap.Controllers.driveID);
  public static final CommandPS5Controller
    operatorController = new CommandPS5Controller(PortMap.Controllers.operatorID);

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

    driverController.L1().whileTrue(
      new IntakeCommand()
    );

    driverController.L2().whileTrue(
      new MotorCommand(Intake.getInstance(), IntakeConstants.ejectPower, 0)
    );
  }
  public Command getAutonomousCommand() {
    return null;
  }
}
