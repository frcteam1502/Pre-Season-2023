// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Motors;
import java.util.ArrayList;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;


public class DriveTrain extends SubsystemBase {

  //Gyro
  private final Gyro gyro = new ADXRS450_Gyro();
  
  //Motors
  private ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>();

  //Swerve kinematics
  private SwerveDriveKinematics kinematics = DriveConstants.swerveDriveKinematics;

  //TODO: Check the VecBuilder values
  public SwerveDrivePoseEstimator teleOdometry = new SwerveDrivePoseEstimator(
    getCurrentAngle(), 
    new Pose2d(), 
    kinematics, 
    VecBuilder.fill(0.1, 0.1, 0.1), 
    VecBuilder.fill(0.5), 
    VecBuilder.fill(0.1, 0.1, 0.1));

  //TODO: Get actual PID values
  public PIDController drivePID = new PIDController(0.1, 0.1, 0.1);
  public PIDController anglePID = new PIDController(0.1, 0.1, 0.1);


  public DriveTrain() {
    modules.add(new SwerveModule(Motors.DRIVE_FRONT_LEFT, Motors.ANGLE_FRONT_LEFT, drivePID, anglePID));
    modules.add(new SwerveModule(Motors.DRIVE_FRONT_RIGHT, Motors.ANGLE_FRONT_RIGHT, drivePID, anglePID));
    modules.add(new SwerveModule(Motors.DRIVE_BACK_LEFT, Motors.ANGLE_BACK_LEFT, drivePID, anglePID));
    modules.add(new SwerveModule(Motors.DRIVE_BACK_RIGHT, Motors.ANGLE_BACK_RIGHT, drivePID, anglePID));
  }

  public void drive(
    double forwardY,
    double strafeX,
    double rotationYAW,
    boolean breaking) {
      if(breaking);
      else {

      }
  }

  @Override
  public void periodic() {
    
  }

  public Rotation2d getCurrentAngle() {
    return gyro.getRotation2d();
  }
}