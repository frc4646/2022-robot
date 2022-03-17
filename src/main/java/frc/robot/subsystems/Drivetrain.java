package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.SparkMaxFactory;
import frc.team4646.Navx;
import frc.team4646.Test;
import frc.team4646.TestMotors.MotorConfig;
import frc.team4646.TestMotors.MotorTestSparkMax;
import frc.team4646.TestMotors.TestConfig;

public class Drivetrain extends SmartSubsystem {
  public static class DataCache {
    public double distanceL, distanceR;
    public double rpmL, rpmR;
    public Rotation2d heading = new Rotation2d();
    public Rotation2d pitch = new Rotation2d();  // Positive is nose up
  }

  private final CANSparkMax masterL, masterR, slaveL, slaveR;
  private final RelativeEncoder encoderL, encoderR;
  private final SparkMaxPIDController pidL, pidR;
  private final Navx gyro;
  private final DifferentialDriveOdometry odometry;
  private DataCache cache = new DataCache();
  
  private boolean isBrakeMode;

  public Drivetrain() {
    masterL = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.DRIVETRAIN_FL, true);
    slaveL = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.CAN.DRIVETRAIN_BL, masterL, false);
    masterR = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.DRIVETRAIN_FR, false);
    slaveR = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.CAN.DRIVETRAIN_BR, masterR, false);
    encoderL = masterL.getEncoder();
    encoderR = masterR.getEncoder();
    pidL = masterL.getPIDController();
    pidR = masterR.getPIDController();
    gyro = new Navx();

    configureMotor(masterL, true, true);
    configureMotor(slaveL, true, false);
    configureMotor(masterR, false, true);
    configureMotor(slaveR, false, false);

    isBrakeMode = true;
    setBrakeMode(false);

    resetEncoders();
    gyro.reset();
    odometry = new DifferentialDriveOdometry(cache.heading);
  }

  public void configureMotor(CANSparkMax motor, boolean isLeft, boolean isMaster) {
    // motor.setInverted(!isLeft);
    motor.enableVoltageCompensation(Constants.DRIVETRAIN.VOLTAGE_COMPENSATION);
    motor.setSmartCurrentLimit(Constants.DRIVETRAIN.CURRENT_LIMIT);
    if (isMaster) {
      motor.getPIDController().setP(isLeft ? Constants.DRIVETRAIN.VELOCITY_L_P : Constants.DRIVETRAIN.VELOCITY_R_P);
      motor.getPIDController().setI(isLeft ? Constants.DRIVETRAIN.VELOCITY_L_I : Constants.DRIVETRAIN.VELOCITY_R_I);
      motor.getPIDController().setD(isLeft ? Constants.DRIVETRAIN.VELOCITY_L_D : Constants.DRIVETRAIN.VELOCITY_R_D);
      motor.getPIDController().setFF(isLeft ? Constants.DRIVETRAIN.VELOCITY_L_F : Constants.DRIVETRAIN.VELOCITY_R_F);
      // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);  // Velocity
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);  // Position TODO try 10
    }
  }

  @Override
  public void cacheSensors() {
    cache.distanceL = encoderL.getPosition();
    cache.distanceR = encoderR.getPosition();
    cache.rpmL = encoderL.getVelocity();
    cache.rpmR = encoderR.getVelocity();
    cache.heading = gyro.getHeading();
    cache.pitch = Rotation2d.fromDegrees(gyro.getPitch());
    odometry.update(getHeading(), rotationsToMeters(cache.distanceL), rotationsToMeters(cache.distanceR));
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    if (showDetails) {
      SmartDashboard.putNumber("Drive: Heading", cache.heading.getDegrees());
      SmartDashboard.putNumber("Drive: Pitch", cache.pitch.getDegrees());
    }
    if (Constants.DRIVETRAIN.TUNING) {
      // SmartDashboard.putNumber("Drive: Distance L", cache.distanceL);
      // SmartDashboard.putNumber("Drive: Distance R", cache.distanceR);
      SmartDashboard.putNumber("Drive: RPM L", cache.rpmL);
      SmartDashboard.putNumber("Drive: RPM R", cache.rpmR);
      SmartDashboard.putNumber("Drive: X", odometry.getPoseMeters().getX());
      SmartDashboard.putNumber("Drive: Y", odometry.getPoseMeters().getY());
    }
  }

  public void resetEncoders() {
    encoderL.setPosition(0.0);
    encoderR.setPosition(0.0);
    // cache = new DataCache();
  }

  public void resetPose(Pose2d pose) {
    resetEncoders();
    gyro.reset();
    odometry.resetPosition(pose, gyro.getHeading());
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
    masterL.set(left/12.0);
    masterR.set(right/12.0);
  }

  public void setClosedLoopVelocity(double wheelSpeedL, double wheelSpeedR) {
    // double feedforwardL = Constants.DRIVETRAIN.FEED_FORWARD.calculate(speeds.leftMetersPerSecond);
    // double feedforwardR = Constants.DRIVETRAIN.FEED_FORWARD.calculate(speeds.rightMetersPerSecond);
    // TODO adjust feed forward using measured gains?
    pidL.setReference(metersToRotations(wheelSpeedL), ControlType.kDutyCycle);
    pidR.setReference(metersToRotations(wheelSpeedR), ControlType.kDutyCycle);
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
    return rotations / Constants.DRIVETRAIN.GEAR_RATIO * Constants.DRIVETRAIN.WHEEL_DIAMETER * Math.PI * 0.0254;
  }
  private double metersToRotations(double meters) {
    return meters * Constants.DRIVETRAIN.GEAR_RATIO / Constants.DRIVETRAIN.WHEEL_DIAMETER / Math.PI / 0.0254;
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
      new TestConfig().amps(5.0, 2.0).rpm(90.0, 200.0, encoderR::getVelocity));
    MotorTestSparkMax.checkMotors(this,
      Arrays.asList(new MotorConfig<>("MasterL", masterL), new MotorConfig<>("SlaveL", slaveL)),
      new TestConfig().amps(5.0, 2.0).rpm(90.0, 200.0, encoderL::getVelocity));
  }
}
