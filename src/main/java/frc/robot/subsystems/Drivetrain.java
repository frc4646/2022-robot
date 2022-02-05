package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;
import frc.team254.drivers.SparkMaxFactory;
import frc.team254.drivers.TalonFXFactory;

public class Drivetrain extends SmartSubsystem {
  public static class DataCache {
    public double distanceL, distanceR;
    public double rpmL, rpmR;
    public Rotation2d heading = new Rotation2d();
  }

  private final CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;
  // private final TalonFX masterL, masterR, slaveL, slaveR;
  private final AHRS gyro;
  private final DifferentialDriveOdometry odometry;
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.Drivetrain.FEED_FORWARD_GAIN_STATIC, Constants.Drivetrain.FEED_FORWARD_GAIN_VELOCITY, Constants.Drivetrain.FEED_FORWARD_GAIN_ACCEL);
  private DataCache cache = new DataCache();
  
  private boolean isBrakeMode;
  private Rotation2d gyroOffset;

  public Drivetrain() {
    leftMaster = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.DRIVETRAIN_FL);
    leftSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.CAN.DRIVETRAIN_BL, leftMaster, false);
    rightMaster = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.DRIVETRAIN_FR);
    rightSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.CAN.DRIVETRAIN_BR, rightMaster, true);
    // masterL = TalonFXFactory.createDefaultTalon(Constants.Ports.DRIVETRAIN_FL);
    // masterR = TalonFXFactory.createDefaultTalon(Constants.Ports.DRIVETRAIN_FR);
    // slaveL = TalonFXFactory.createPermanentSlaveTalon(Constants.Ports.DRIVETRAIN_BL, masterL, false);
    // slaveR = TalonFXFactory.createPermanentSlaveTalon(Constants.Ports.DRIVETRAIN_BR, masterR, true);
    gyro = new AHRS();

    configureMotor(leftMaster, true, true);
    configureMotor(leftSlave, true, false);
    configureMotor(rightMaster, false, true);
    configureMotor(rightSlave, false, false);

    isBrakeMode = true;
    setBrakeMode(false);

    resetEncoders();
    gyro.reset();
    odometry = new DifferentialDriveOdometry(cache.heading);
  }

  public void configureMotor(CANSparkMax motor, boolean isLeft, boolean isMaster) {
    motor.setInverted(!isLeft);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT); // TODO find more examples to confirm what values are best
    motor.setOpenLoopRampRate(.1);  // TODO is this a good idea?
    if (isMaster) {
      motor.getPIDController().setP(Constants.Drivetrain.P);
      motor.getPIDController().setI(Constants.Drivetrain.I);
      motor.getPIDController().setD(Constants.Drivetrain.D);
      motor.getPIDController().setFF(Constants.Drivetrain.F);
    }
  }

  @Override
  public void cacheSensors() {
    cache.distanceL = leftMaster.getEncoder().getPosition();
    cache.distanceR = rightMaster.getEncoder().getPosition();
    cache.rpmL = leftMaster.getEncoder().getVelocity();
    cache.rpmR = rightMaster.getEncoder().getVelocity();
    cache.heading = Rotation2d.fromDegrees(gyro.getFusedHeading()).rotateBy(gyroOffset);
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Drive Distance L", cache.distanceL);
    SmartDashboard.putNumber("Drive Distance R", cache.distanceR);
    SmartDashboard.putNumber("Drive RPM L", cache.rpmL);
    SmartDashboard.putNumber("Drive RPM R", cache.rpmR);
    SmartDashboard.putNumber("Heading", cache.heading.getDegrees());
  }

  public void resetEncoders() {
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

  public void setHeading(Rotation2d heading) {
    gyroOffset = heading.rotateBy(Rotation2d.fromDegrees(gyro.getFusedHeading()).rotateBy(Rotation2d.fromDegrees(180.0)));
    cache.heading = heading;
  }

  public double getRPM(){
    return (cache.rpmL + cache.rpmR) / 2.0;
  }

  public Rotation2d getHeading() {
    return cache.heading;
  }

  public void updateOdometry() {
    // TODO m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  @Override
  public void runTests() {
    Test.checkFirmware(new Test.FirmwareSparkMax(this, leftMaster));
    Test.checkFirmware(new Test.FirmwareSparkMax(this, rightMaster));
    Test.checkFirmware(new Test.FirmwareSparkMax(this, leftSlave));
    Test.checkFirmware(new Test.FirmwareSparkMax(this, rightSlave));
  }
}
