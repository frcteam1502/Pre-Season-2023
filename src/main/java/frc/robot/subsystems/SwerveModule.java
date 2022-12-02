package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  public CANSparkMax driveMotor, angleMotor;
  public PIDController drivePID, anglePID;

  public SwerveModule(CANSparkMax driveMotor, CANSparkMax angleMotor, PIDController drivePID, PIDController anglePID) {
    this.angleMotor = angleMotor;
    this.driveMotor = driveMotor;

    this.angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    this.drivePID = drivePID;
    this.anglePID = anglePID;

    this.drivePID.
  }

  @Override
  public void periodic() {}

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), Rotation2d.fromDegrees(driveMotor.getEncoder().getPosition()));
  }

  public void resetEncoders() {
    angleMotor.getEncoder().setPosition(0);
    driveMotor.getEncoder().setPosition(0);
  }

  public void toAngle(double goalAngle) {

  }

  public void toSpeed(double goalSpeed) {

  }

  public void setModule(double goalSpeed, double goalAngle) {
    toSpeed(goalSpeed);
    toAngle(goalAngle);
  }
}
