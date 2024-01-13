package frc.robot.automations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class EjectAutomation extends SequentialCommandGroup{
    public EjectAutomation() {
        addCommands(
            new InstantCommand(() -> Shooter.getInstance().setSetPoint(ShooterConstants.ejectPower)),
            new WaitUntilCommand(() -> Shooter.getInstance().atPoint()),
            new MotorCommand(Intake.getInstance(), () -> IntakeConstants.ejectPower, () -> IntakeConstants.ejectPower),
            new WaitCommand(0.3),
            new MotorCommand(Intake.getInstance(), 0.0, 0.0),
            new InstantCommand(() -> Shooter.getInstance().setSetPoint(0))
        );
    }
}