package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.Shooter;

public class SetShooter extends SequentialCommandGroup {
    public void setShooter(double setPoint) {
        addCommands(
            new InstantCommand(() -> Shooter.getInstance().setSetPoint(setPoint)),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(Shooter.getInstance()::atPoint)
            )
        );
    }  
}
