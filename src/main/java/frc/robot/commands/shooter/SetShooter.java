// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooter extends SequentialCommandGroup {

  public SetShooter(Supplier<Double> upperSetPoint, Supplier<Double> lowerSetpoint) {
        addCommands(
            new InstantCommand(() -> UpperShooter.getInstance()
                .setSetPoint(upperSetPoint)), 
            new InstantCommand(() -> LowerShooter.getInstance()
                .setSetPoint(lowerSetpoint)),
            new WaitForSetPoint()
        );
    }
}
