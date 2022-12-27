package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.utils.AngleUtils;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleSparkMax extends SubsystemBase {
  public final CANSparkMax driveMotor;
  public final CANSparkMax turnMotor;

  public final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  private final SparkMaxPIDController drivePIDController;
  private SparkMaxPIDController m_turnSMController = null;
  private PIDController turnPIDController = null;

  public ModulePosition modulePosition;
  SwerveModuleState state;
  public int moduleNumber;
  public String[] modAbrev = { "_FL", "_FR", "_RL", "_RR" };
  String driveLayout;
  String turnLayout;
  String canCoderLayout;
  Pose2d m_pose;
  double testAngle;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      ModuleConstants.ksVolts,
      ModuleConstants.kvVoltSecondsPerMeter,
      ModuleConstants.kaVoltSecondsSquaredPerMeter);

  private double m_lastAngle;
  public double angle;

  public double m_turningEncoderOffset;

  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 1;
  private final int SIM_SLOT = 2;

  public double actualAngleDegrees;

  private double angleDifference;
  private double angleIncrementPer20ms;
  private double tolDegPerSec = .05;
  private double toleranceDeg = .25;
  public boolean driveMotorConnected;
  public boolean turnMotorConnected;
  public boolean turnCoderConnected;
  private boolean useRRPid = true;
  private double turnDeadband = .5;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel       The channel of the drive motor.
   * @param turningMotorChannel     The channel of the turning motor.
   * @param driveEncoderChannels    The channels of the drive encoder.
   * @param turningCANCoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed    Whether the drive encoder is reversed.
   * @param turningEncoderReversed  Whether the turning encoder is reversed.
   * @param turningEncoderOffset
   */
  public SwerveModuleSparkMax(
      ModulePosition modulePosition,
      int driveMotorCanChannel,
      int turningMotorCanChannel,
      int cancoderCanChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      double turningEncoderOffset) {

    driveMotor = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turningMotorCanChannel, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    driveMotor.setSmartCurrentLimit(20);
    turnMotor.setSmartCurrentLimit(20);

    driveMotor.enableVoltageCompensation(DriveConstants.kVoltCompensation);
    turnMotor.enableVoltageCompensation(DriveConstants.kVoltCompensation);

    /*FIXME: Absolute Encoder
    m_turnCANcoder = new CTRECanCoder(cancoderCanChannel);
    m_turnCANcoder.configFactoryDefault();
    m_turnCANcoder.configAllSettings(AngleUtils.generateCanCoderConfig());
    */
    m_turningEncoderOffset = turningEncoderOffset;

    driveMotor.setInverted(driveMotorReversed);

    turnMotor.setInverted(turningMotorReversed);

    driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
    turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);
    turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncRPMperMPS);

    drivePIDController = driveMotor.getPIDController();

    if (RobotBase.isReal()) {
      drivePIDController.setP(.01, VEL_SLOT);
      drivePIDController.setD(0, VEL_SLOT);
      drivePIDController.setI(0, VEL_SLOT);
      drivePIDController.setIZone(1, VEL_SLOT);
    } else drivePIDController.setP(1, SIM_SLOT);

    turnEncoder = turnMotor.getEncoder();
    turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningDegreesPerEncRev);
    turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningDegreesPerEncRev / 60);

    turnPIDController = new PIDController(ModuleConstants.kPModuleTurnController, 0, 0); //FIXME: ???? Does this actually work ????


    this.modulePosition = modulePosition;

    moduleNumber = modulePosition.ordinal();

    if (RobotBase.isSimulation()) REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));

    //resetAngleToAbsolute(); FIXME: Absolute Encoder
  }

  @Override
  public void periodic() {
    /* FIXME: Absolute Encoder
    if (m_turnCANcoder.getFaulted()) {
      // SmartDashboard.putStringArray("CanCoderFault"
      // + m_modulePosition.toString(), m_turnCANcoder.getFaults());
      SmartDashboard.putStringArray("CanCoderStickyFault"
          + modulePosition.toString(), m_turnCANcoder.getStickyFaults());
    } */
    SmartDashboard.putNumber("steer enc [" + modulePosition.toString() + "]", turnEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {

    REVPhysicsSim.getInstance().run();

  }

  public SwerveModuleState getState() {

    if (RobotBase.isReal())

      return new SwerveModuleState(driveEncoder.getVelocity(), getHeadingRotation2d());

    else

      return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(angle));
  }

  public ModulePosition getModulePosition() {

    return modulePosition;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

    state = AngleUtils.optimize(desiredState, getHeadingRotation2d());

    // state = SwerveModuleState.optimize(desiredState, new
    // Rotation2d(actualAngleDegrees));

    // turn motor code
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angle = (Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))

        ? m_lastAngle

        : state.angle.getDegrees();

    m_lastAngle = angle;

    if (RobotBase.isReal()) {

      // turn axis

      actualAngleDegrees = turnEncoder.getPosition();

      if (useRRPid) {

        positionTurn(angle);
      }

      else {

        positionSMTurn(angle);
      }

      // drive axis

      if (isOpenLoop)

        driveMotor.set(state.speedMetersPerSecond / ModuleConstants.kFreeMetersPerSecond);

      else {

        drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, VEL_SLOT);

      }
    }

    if (RobotBase.isSimulation()) {

      drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, SIM_SLOT);

      // no simulation for angle - angle command is returned directly to drive
      // subsystem as actual angle in 2 places - getState() and getHeading

      simTurnPosition(angle);
    }

  }

  public static double limitMotorCmd(double motorCmdIn) {
    return Math.max(Math.min(motorCmdIn, 1.0), -1.0);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(0);

  }

  public double getHeadingDegrees() {

    if (RobotBase.isReal())

      return turnEncoder.getPosition();

    else

      return actualAngleDegrees;

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public void setDriveBrakeMode(boolean on) {
    if (on)
      driveMotor.setIdleMode(IdleMode.kBrake);
    else
      driveMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean on) {
    if (on)
      turnMotor.setIdleMode(IdleMode.kBrake);
    else
      turnMotor.setIdleMode(IdleMode.kCoast);
  }

  public void zeroModule() {
    //resetAngleToAbsolute(); FIXME: Add back please
    driveEncoder.setPosition(0.0);
  }

  /*FIXME: Absolute Encoder
  public void resetAngleToAbsolute() {
    double angle = m_turnCANcoder.getAbsolutePosition() - m_turningEncoderOffset;
    turnEncoder.setPosition(angle);
  }
  */
  public double getTurnAngle() {
    return turnEncoder.getPosition();
  }

  public void turnMotorMove(double speed) {
    turnMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public void positionSMTurn(double angle) {

    m_turnSMController.setReference(angle, ControlType.kPosition, POS_SLOT);
  }

  public void positionTurn(double angle) {

    double turnAngleError = Math.abs(angle - turnEncoder.getPosition());

    double pidOut = turnPIDController.calculate(turnEncoder.getPosition(), angle);
    // if robot is not moving, stop the turn motor oscillating
    if (turnAngleError < turnDeadband

        && Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))

      pidOut = 0;

    turnMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());

  }

  private void simTurnPosition(double angle) {
    
    if (angle != actualAngleDegrees && angleIncrementPer20ms == 0) {

      angleDifference = angle - actualAngleDegrees;

      angleIncrementPer20ms = angleDifference / 20;// 10*20ms = .2 sec move time
    }

    if (angleIncrementPer20ms != 0) {

      actualAngleDegrees += angleIncrementPer20ms;

      if ((Math.abs(angle - actualAngleDegrees)) < .1) {

        actualAngleDegrees = angle;

        angleIncrementPer20ms = 0;
      }
    }
  }

  public void driveMotorMove(double speed) {
    driveMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getDriveCurrent() {
    return driveMotor.getOutputCurrent();
  }

  public double getTurnVelocity() {
    return turnEncoder.getVelocity();
  }

  public double getTurnPosition() {
    return turnEncoder.getPosition();
  }

  public double getTurnCurrent() {
    return turnMotor.getOutputCurrent();
  }

  public boolean turnInPosition(double targetAngle) {
    return Math.abs(targetAngle - getTurnAngle()) < toleranceDeg;
  }

  public boolean turnIsStopped() {

    return Math.abs(turnEncoder.getVelocity()) < tolDegPerSec;
  }

  

}