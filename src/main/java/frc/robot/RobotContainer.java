// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class RobotContainer {

  public static final CommandPS5Controller
    driverController = new CommandPS5Controller(PortMap.Controllers.driveID);
  public static final CommandPS5Controller
    operatorController = new CommandPS5Controller(PortMap.Controllers.operatorID);

  public RobotContainer() {
    NamedCommands.registerCommand("wait", new WaitCommand(3));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return SwerveDrivetrainSubsystem.getInstance()
    .getAutonomousPathCommand("Example Path");
  }
}
