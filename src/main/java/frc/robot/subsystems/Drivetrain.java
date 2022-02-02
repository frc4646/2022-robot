package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Drivetrain extends SmartSubsystem {
  public static class DataCache {
    public double distanceL;
    public double distanceR;
    public double rpmL;
    public double rpmR;
    public Rotation2d heading = new Rotation2d();
  }

  private final CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;
  // private final PigeonIMU gyro;
  // private final Encoder leftEncoder, rightEncoder;
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.Drivetrain.FEED_FORWARD_GAIN_STATIC, Constants.Drivetrain.FEED_FORWARD_GAIN_VELOCITY, Constants.Drivetrain.FEED_FORWARD_GAIN_ACCEL);
  private DataCache cache = new DataCache();
  
  private boolean isBrakeMode;

  public Drivetrain() {
    leftMaster = new CANSparkMax(Constants.Ports.DRIVETRAIN_FL, MotorType.kBrushless);
    leftSlave = new CANSparkMax(Constants.Ports.DRIVETRAIN_BL, MotorType.kBrushless);
    rightMaster = new CANSparkMax(Constants.Ports.DRIVETRAIN_FR, MotorType.kBrushless);
    rightSlave = new CANSparkMax(Constants.Ports.DRIVETRAIN_BR, MotorType.kBrushless);
    // leftEncoder = new Encoder(Constants.Digital.DRIVETRAIN_L_ENCODER_A, Constants.Digital.DRIVETRAIN_L_ENCODER_B, false);
    // rightEncoder = new Encoder(Constants.Digital.DRIVETRAIN_R_ENCODER_A, Constants.Digital.DRIVETRAIN_R_ENCODER_B, true);
    // gyro = new PigeonIMU(Constants.Ports.GYRO);
    
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    configureMotor(leftMaster, true);
    configureMotor(leftSlave, true);
    configureMotor(rightMaster, false);
    configureMotor(rightSlave, false);
    // gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10, 10);

    isBrakeMode = true;
    setBrakeMode(false);

    // leftEncoder.setReverseDirection(true);
    // rightEncoder.setReverseDirection(false);
    resetEncoders();
  }

  @Override
  public void cacheSensors() {
    cache.distanceL = leftMaster.getEncoder().getPosition();
    cache.distanceR = rightMaster.getEncoder().getPosition();
    cache.rpmL = leftMaster.getEncoder().getVelocity();
    cache.rpmR = rightMaster.getEncoder().getVelocity();
    // cached.heading = Rotation2d.fromDegrees(gyro.getFusedHeading()).rotateBy(mGyroOffset);
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Drive Distance L", cache.distanceL);
    SmartDashboard.putNumber("Drive Distance R", cache.distanceR);
    SmartDashboard.putNumber("Drive RPM L", cache.rpmL);
    SmartDashboard.putNumber("Drive RPM R", cache.rpmR);
  }

  public void configureMotor(CANSparkMax motor, boolean isLeft) {
    motor.setInverted(!isLeft);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT); // TODO find more examples to confirm what values are best
    motor.setOpenLoopRampRate(.25);
  }

  public void resetEncoders() {
    // leftEncoder.reset();
    // rightEncoder.reset();
    leftMaster.getEncoder().setPosition(0.0);
    rightMaster.getEncoder().setPosition(0.0);
    cache = new DataCache();
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode != enable) {
      IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
      leftMaster.setIdleMode(mode);
      leftSlave.setIdleMode(mode);
      rightMaster.setIdleMode(mode);
      rightSlave.setIdleMode(mode);
      isBrakeMode = enable;
    }
  }

  public void setOpenLoop(double left, double right) {
    leftMaster.set(left);
    rightMaster.set(right);
  }

  public void setClosedLoopVelocity(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedForward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedForward.calculate(speeds.rightMetersPerSecond);
    // TODO set each master closed loop setpoint + arbitrary feedforward
  }

  public Rotation2d getHeading() {
    return cache.heading;
  }

  public void updateOdometry() {
    // TODO m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
}
