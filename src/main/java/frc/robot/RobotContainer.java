package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveByJoysticks;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.Joysticks;;

public class RobotContainer {
  //Drive Subsystem and Command
  private final DriveTrain driveTrain = new DriveTrain();
  private final DriveByJoysticks drive = 
  new DriveByJoysticks(
    driveTrain,
    Joysticks.DRIVE_CONTROLLER.getLeftY(),
    Joysticks.DRIVE_CONTROLLER.getLeftX(),
    Joysticks.DRIVE_CONTROLLER.getRightX(),
    Joysticks.DRIVE_CONTROLLER.getRightBumper()
    );

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    driveTrain.setDefaultCommand(drive);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
