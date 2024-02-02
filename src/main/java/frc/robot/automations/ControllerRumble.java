// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.automations;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
// import frc.robot.RobotContainer;

// public class ControllerRumble extends Command {
//   /** Creates a new ControllerRumble. */
//   private CommandPS5Controller driverController;
//   private CommandPS5Controller operaController;
//   public double rumbleTime;
//   public boolean driverRumble;
//   public boolean operatorRumble;
//   public double rumblePower;
//   public Timer stopwatch;

//   public ControllerRumble(double time , double power ,boolean drive , boolean operator) {
//    driverController = RobotContainer.driverController;
//    operaController = RobotContainer.operatorController;
//    rumbleTime = time;
//    driverRumble = drive;
//    operatorRumble = operator;
//    rumblePower = power;
//   }

//   public ControllerRumble(double time) {
//     driverController = RobotContainer.driverController;
//     operaController = RobotContainer.operatorController;
//     rumbleTime = time;
//     driverRumble = true;
//     operatorRumble = false;
//     rumblePower = 1;
//   }

//   public void rumbleControllers(boolean driver , boolean operator) {
//     if (driver) {
//       driverController.getHID().setRumble(RumbleType.kBothRumble, rumblePower);
//     }

//     if (operator) {
//       operaController.getHID().setRumble(RumbleType.kBothRumble, rumblePower);
//     }
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     stopwatch.reset();
//     stopwatch.start();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     rumbleControllers(driverRumble, operatorRumble);
//     System.out.println("rumble");
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
//     operaController.getHID().setRumble(RumbleType.kBothRumble, 0);
//     stopwatch.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return rumbleTime >= stopwatch.get() ;
//   }
// }
