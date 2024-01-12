package frc.robot.commands.Intake;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommand extends Command {
  private Intake intake;
  private MotorCommand intakeCommand;
  
  public IntakeCommand() {
    intake = Intake.getInstance();
    intakeCommand = new MotorCommand(intake, IntakeConstants.intakePower, 0);
  }

  @Override
  public void initialize() {
    intakeCommand.initialize();
  }

  @Override
  public void execute() {
    intakeCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    intakeCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return intake.isGamePiece();
  }
}
