package frc.robot.commands.Intake;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommand extends Command {
  private Intake intake;
  private MotorCommand motorCommand;
  
  public IntakeCommand() {
    intake = Intake.getInstance();
    motorCommand = new MotorCommand(intake, IntakeConstants.intakePower, 0);
  }

  @Override
  public void initialize() {
    motorCommand.initialize();
  }

  @Override
  public void execute() {
    motorCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    motorCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return intake.isGamePieceInIntake();
  }
}
