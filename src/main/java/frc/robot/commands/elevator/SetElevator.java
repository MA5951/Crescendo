package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevator extends SequentialCommandGroup {
    
    public SetElevator(double setPoint) {
        addCommands(
            new InstantCommand(() -> Elevator.getInstance().setSetPoint(setPoint)),
            new WaitUntilCommand(Elevator.getInstance()::atPoint)
        );
    }
}