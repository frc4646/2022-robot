package frc.robot.subsystems;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;
import frc.robot.util.TestMotors.MotorConfig;
import frc.robot.util.TestMotors.MotorTestSparkMax;
import frc.robot.util.TestMotors.TestConfig;
import frc.team254.drivers.SparkMaxFactory;

public class Drivetrain extends SmartSubsystem {
  public static class DataCache {
    public double distanceL, distanceR;
    public double rpmL, rpmR;
    public Rotation2d heading = new Rotation2d();
    public Rotation2d pitch = new Rotation2d();  // Positive is nose up
  }

  private final CANSparkMax masterL, masterR, slaveL, slaveR;
  private final AHRS gyro;
  private final DifferentialDriveOdometry odometry;
  private DataCache cache = new DataCache();
  
  private boolean isBrakeMode;
  private Rotation2d gyroOffset = Rotation2d.fromDegrees(0.0);

  public Drivetrain() {
    masterL = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.DRIVETRAIN_FL, true);
    slaveL = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.CAN.DRIVETRAIN_BL, masterL, false);
    masterR = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.DRIVETRAIN_FR, false);
    slaveR = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.CAN.DRIVETRAIN_BR, masterR, false);
    gyro = new AHRS();

    configureMotor(masterL, true, true);
    configureMotor(slaveL, true, false);
    configureMotor(masterR, false, true);
    configureMotor(slaveR, false, false);
    masterL.getPIDController().setP(Constants.DRIVETRAIN.P_LEFT);
    masterL.getPIDController().setI(Constants.DRIVETRAIN.I_LEFT);
    masterL.getPIDController().setD(Constants.DRIVETRAIN.D_LEFT);
    masterL.getPIDController().setFF(Constants.DRIVETRAIN.F_LEFT);
    masterR.getPIDController().setP(Constants.DRIVETRAIN.P_RIGHT);
    masterR.getPIDController().setI(Constants.DRIVETRAIN.I_RIGHT);
    masterR.getPIDController().setD(Constants.DRIVETRAIN.D_RIGHT);
    masterR.getPIDController().setFF(Constants.DRIVETRAIN.F_RIGHT);

    isBrakeMode = true;
    setBrakeMode(false);

    resetEncoders();
    gyro.reset();
    odometry = new DifferentialDriveOdometry(cache.heading);
  }

  public void configureMotor(CANSparkMax motor, boolean isLeft, boolean isMaster) {
    // motor.setInverted(!isLeft);
    motor.enableVoltageCompensation(Constants.DRIVETRAIN.VOLTAGE_COMPENSATION);
    motor.setSmartCurrentLimit(Constants.DRIVETRAIN.CURRENT_LIMIT); // TODO find more examples to confirm what values are best
    // TODO faster status frames
  }

  @Override
  public void cacheSensors() {
    cache.distanceL = masterL.getEncoder().getPosition();
    cache.distanceR = masterR.getEncoder().getPosition();
    cache.rpmL = masterL.getEncoder().getVelocity();
    cache.rpmR = masterR.getEncoder().getVelocity();
    cache.heading = Rotation2d.fromDegrees(gyro.getFusedHeading()).rotateBy(gyroOffset);
    cache.pitch = Rotation2d.fromDegrees(gyro.getPitch());
    odometry.update(getHeading(), rotationsToMeters(cache.distanceL), rotationsToMeters(cache.distanceR));
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Drive: Heading", cache.heading.getDegrees());
    SmartDashboard.putNumber("Drive: Pitch", cache.pitch.getDegrees());
    if (Constants.DRIVETRAIN.TUNING) {
      SmartDashboard.putNumber("Drive: Distance L", cache.distanceL);
      SmartDashboard.putNumber("Drive: Distance R", cache.distanceR);
      SmartDashboard.putNumber("Drive: RPM L", cache.rpmL);
      SmartDashboard.putNumber("Drive: RPM R", cache.rpmR);
    }
  }

  public void resetEncoders() {
    masterL.getEncoder().setPosition(0.0);
    masterR.getEncoder().setPosition(0.0);
    cache = new DataCache();
  }

  public void resetPose(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode == enable) {
      return;  // Already in this mode
    }
    IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
    masterL.setIdleMode(mode);
    slaveL.setIdleMode(mode);
    masterR.setIdleMode(mode);
    slaveR.setIdleMode(mode);
    isBrakeMode = enable;
  }

  public void setOpenLoop(double left, double right) {
    masterL.set(left);
    masterR.set(right);
  }

  public void setVolts(double left, double right) {
    masterL.getPIDController().setReference(left, ControlType.kVoltage);
    masterR.getPIDController().setReference(right, ControlType.kVoltage);
  }

  public void setClosedLoopVelocity(DifferentialDriveWheelSpeeds speeds) {
    double feedforwardL = Constants.DRIVETRAIN.FEED_FORWARD.calculate(speeds.leftMetersPerSecond);
    double feedforwardR = Constants.DRIVETRAIN.FEED_FORWARD.calculate(speeds.rightMetersPerSecond);
    masterL.getPIDController().setReference(feedforwardL, ControlType.kDutyCycle);
    masterR.getPIDController().setReference(feedforwardR, ControlType.kDutyCycle);
  }

  public void setHeading(Rotation2d heading) {
    // gyroOffset = heading.rotateBy(Rotation2d.fromDegrees(gyro.getFusedHeading()).rotateBy(Rotation2d.fromDegrees(180.0)));
    cache.heading = heading;
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(rotationsToMeters(cache.rpmL), rotationsToMeters(cache.rpmR));
  }

  public Pose2d getPose() { return odometry.getPoseMeters(); }
  public double getDistance() { return (cache.distanceL + cache.distanceR) / 2.0; }
  public double getRPM(){ return (cache.rpmL + cache.rpmR) / 2.0; }
  public Rotation2d getHeading() { return cache.heading; }
  public Rotation2d getPitch() { return cache.pitch; }

  private double rotationsToMeters(double rotations) {
    return rotations * Constants.DRIVETRAIN.WHEEL_DIAMETER * Math.PI * 0.0254;
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, masterR);
    Test.checkFirmware(this, slaveL);
    Test.checkFirmware(this, slaveR);
    Test.add(this, "Gyro - Pitch (Robot Is Flat)", Math.abs(cache.pitch.getDegrees()) < 5.0);

    setBrakeMode(false);
    MotorTestSparkMax.checkMotors(this,
      Arrays.asList(new MotorConfig<>("MasterR", masterR), new MotorConfig<>("SlaveR", slaveR)),
      new TestConfig().amps(5.0, 2.0).rpm(90.0, 200.0, masterR.getEncoder()::getVelocity));
    MotorTestSparkMax.checkMotors(this,
      Arrays.asList(new MotorConfig<>("MasterL", masterL), new MotorConfig<>("SlaveL", slaveL)),
      new TestConfig().amps(5.0, 2.0).rpm(90.0, 200.0, masterL.getEncoder()::getVelocity));
  }
}
