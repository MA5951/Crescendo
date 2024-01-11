package frc.robot.commands.Intake;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommand extends Command {
  private Intake intake;
  private MotorCommand intakeMotorCommand;
  
  private Conveyor conveyor;
  private MotorCommand conveyorMotorCommand;

  public IntakeCommand() {
    intake = Intake.getInstance();
    intakeMotorCommand = new MotorCommand(intake, IntakeConstants.intakePower, 0);

    conveyor = Conveyor.getInstance();
    conveyorMotorCommand = new MotorCommand(conveyor, ConveyorConstants.conveyorPower, 0);
  }

  @Override
  public void initialize() {
    intakeMotorCommand.initialize();
    conveyorMotorCommand.initialize();
  }

  @Override
  public void execute() {
    intakeMotorCommand.execute();
    conveyorMotorCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    intakeMotorCommand.end(interrupted);
    conveyorMotorCommand.end(interrupted);

    if (!interrupted) {
      intake.setPiceIn(true);
    }
  }

  @Override
  public boolean isFinished() {
    return false;//intake.getSensor() || intake.isPiceIn();
  }
}
