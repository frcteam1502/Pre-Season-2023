package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveByJoysticks extends CommandBase {
  private final DriveTrain drive;
  private final SlewRateLimiter slewX = new SlewRateLimiter(DriveConstants.kTranslationSlew);
  private final SlewRateLimiter slewY = new SlewRateLimiter(DriveConstants.kTranslationSlew);
  private final SlewRateLimiter slewRot = new SlewRateLimiter(DriveConstants.kRotationSlew);
  private final double throttle, strafe, rotation;
  private final boolean lockWheels;

  public DriveByJoysticks(
    DriveTrain drive, 
    double throttle, 
    double strafe, 
    double rotation, 
    boolean lockWheels ) 
  {
    this.drive = drive;
    this.throttle = throttle;
    this.strafe = strafe;
    this.rotation = rotation;
    this.lockWheels = lockWheels;
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double throttle = slewX.calculate(this.throttle);
    double strafe = slewY.calculate(this.strafe);
    double rotation = slewRot.calculate(this.rotation);
    drive.drive(throttle, strafe, rotation, lockWheels, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
