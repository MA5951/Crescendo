// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;

public class RunShoot extends Command {
  private Command shoot;
  private boolean isPoduim;

  private boolean cantAdjust() {
    if (isPoduim) {
      return false;
    }
    return Shoot.xDis > SwerveConstants.maxSpeakerDistanceX || 
            Shoot.yDis > SwerveConstants.maxSpeakerDistanceY;
  }

  public RunShoot(boolean isPoduim) {
    this.isPoduim = isPoduim;
    shoot = new Shoot(isPoduim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoot.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shoot.isFinished() || cantAdjust();
  }
}
