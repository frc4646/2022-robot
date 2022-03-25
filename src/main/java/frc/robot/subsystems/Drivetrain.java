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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.SparkMaxFactory;
import frc.team4646.Navx;
import frc.team4646.Test;
import frc.team4646.TestMotors.MotorConfig;
import frc.team4646.TestMotors.MotorTestSparkMax;
import frc.team4646.TestMotors.TestConfig;

public class Drivetrain extends SmartSubsystem {
  private class DataCache {
    public double distanceL, distanceR;
    public double rpmL, rpmR;
    public Rotation2d heading = new Rotation2d();
    public Rotation2d pitch = new Rotation2d();  // Positive is nose up
    public double timestampLast;
    public double headingRate;
    private boolean inBrakeMode = true;
  }
  private class OutputCache {
    public ControlType mode = ControlType.kDutyCycle;
    public double setpointL = 0.0, setpointR = 0.0;

    public void set(ControlType mode, double setpointL, double setpointR) {
      outputs.mode = mode;
      outputs.setpointL = setpointL;
      outputs.setpointR = setpointR;
    }
  }

  private final CANSparkMax masterL, masterR, slaveL, slaveR;
  private final RelativeEncoder encoderL, encoderR;
  private final SparkMaxPIDController pidL, pidR;
  private final Navx gyro;
  private final DifferentialDriveOdometry odometry;
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();

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

    setBrakeMode(!cache.inBrakeMode);
    resetEncoders();
    gyro.reset();
    odometry = new DifferentialDriveOdometry(cache.heading);
  }

  public void configureMotor(CANSparkMax motor, boolean isLeft, boolean isMaster) {
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(Constants.DRIVETRAIN.CURRENT_LIMIT);
    if (isMaster) {
      SparkMaxPIDController pid = isLeft ? pidL : pidR;
      pid.setP(Constants.DRIVETRAIN.VELOCITY_P);
      pid.setI(Constants.DRIVETRAIN.VELOCITY_I);
      pid.setD(Constants.DRIVETRAIN.VELOCITY_D);
      pid.setFF(Constants.DRIVETRAIN.VELOCITY_F);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);  // Velocity
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);  // Position
    }
  }

  @Override
  public void cacheSensors() {
    cache.distanceL = encoderL.getPosition();
    cache.distanceR = encoderR.getPosition();
    cache.rpmL = encoderL.getVelocity();
    cache.rpmR = encoderR.getVelocity();
    cache.pitch = Rotation2d.fromDegrees(gyro.getPitch());
    double now = Timer.getFPGATimestamp();
    double dt = now - cache.timestampLast;
    Rotation2d headingNow = gyro.getHeading();
    cache.headingRate = headingNow.minus(cache.heading).getDegrees() / dt; //gyro.getHeadingRate();
    cache.heading = headingNow;
    cache.timestampLast = now;
    odometry.update(getHeading(), rotationsToMeters(cache.distanceL), rotationsToMeters(cache.distanceR));
  }

  @Override
  public void updateHardware() {
    updateMotors();
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    if (showDetails) {
      SmartDashboard.putNumber("Drive: Heading", cache.heading.getDegrees());
      SmartDashboard.putNumber("Drive: Pitch", cache.pitch.getDegrees());
      SmartDashboard.putNumber("Drive: Heading Rate", cache.headingRate);
      // SmartDashboard.putString("Drive: Command", this.getCurrentCommand().getName());
    }
    if (Constants.TUNING.DRIVETRAIN) {
      DifferentialDriveWheelSpeeds speed = getWheelSpeeds();
      SmartDashboard.putNumber("Drive: Velocity", (speed.leftMetersPerSecond + speed.rightMetersPerSecond) / 2.0);
      SmartDashboard.putNumber("Drive: Error L", outputs.setpointL - speed.leftMetersPerSecond);
      SmartDashboard.putNumber("Drive: Error R", outputs.setpointR - speed.rightMetersPerSecond);
      SmartDashboard.putNumber("Drive: X", odometry.getPoseMeters().getX());
      SmartDashboard.putNumber("Drive: Y", odometry.getPoseMeters().getY());
    }
  }

  public void setOpenLoop(double percentL, double percentR) { outputs.set(ControlType.kDutyCycle, percentL, percentR); }
  public void setVolts(double voltsL, double voltsR) { outputs.set(ControlType.kDutyCycle, voltsL / 12.0, voltsR / 12.0); }
  public void setClosedLoopVelocity(double speedL, double speedR) { outputs.set(ControlType.kVelocity, speedL, speedR); }

  public void resetEncoders() {
    encoderL.setPosition(0.0);
    encoderR.setPosition(0.0);
    cache.distanceL = 0.0;
    cache.distanceR = 0.0;
  }

  public void resetPose(Pose2d pose) {
    resetEncoders();
    gyro.reset();
    cache.timestampLast = Timer.getFPGATimestamp();
    cache.heading = gyro.getHeading();
    odometry.resetPosition(pose, cache.heading);
  }

  public void setBrakeMode(boolean enable) {
    if (cache.inBrakeMode != enable) {
      cache.inBrakeMode = enable;
      IdleMode mode = cache.inBrakeMode ? IdleMode.kBrake : IdleMode.kCoast;
      masterL.setIdleMode(mode);
      slaveL.setIdleMode(mode);
      masterR.setIdleMode(mode);
      slaveR.setIdleMode(mode);
    }
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {return new DifferentialDriveWheelSpeeds(rotationsToMeters(cache.rpmL / 60.0), rotationsToMeters(cache.rpmR / 60.0));  }
  public Pose2d getPose() { return odometry.getPoseMeters(); }
  public Rotation2d getHeading() { return cache.heading; }
  public double getHeadingRate() { return cache.headingRate; }
  public Rotation2d getPitch() { return cache.pitch; }

  private double rotationsToMeters(double rotations) { return rotations / Constants.DRIVETRAIN.GEAR_RATIO * Constants.DRIVETRAIN.WHEEL_DIAMETER * Math.PI * 0.0254; }
  private double metersToRotations(double meters) { return meters * Constants.DRIVETRAIN.GEAR_RATIO / Constants.DRIVETRAIN.WHEEL_DIAMETER / Math.PI / 0.0254; }

  private void updateMotors() {
    // double feedforwardL = Math.signum(outputs.setpointL) * Constants.DRIVETRAIN.CRACKPOINT;
    // double feedforwardR = Math.signum(outputs.setpointR) * Constants.DRIVETRAIN.CRACKPOINT;
    if (outputs.mode == ControlType.kVelocity) {
      pidL.setReference(metersToRotations(outputs.setpointL * 60.0), ControlType.kVelocity);
      pidR.setReference(metersToRotations(outputs.setpointR * 60.0), ControlType.kVelocity);
      // pidL.setReference(metersToRotations(outputs.setpointL * 60.0), ControlType.kVelocity, 0, feedforwardL, ArbFFUnits.kPercentOut);
      // pidR.setReference(metersToRotations(outputs.setpointR * 60.0), ControlType.kVelocity, 0, feedforwardR, ArbFFUnits.kPercentOut);
    } else {
      masterL.set(outputs.setpointL);
      masterR.set(outputs.setpointR);
    }
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, masterR);
    Test.checkFirmware(this, slaveL);
    Test.checkFirmware(this, slaveR);
    Test.add(this, "Gyro - Pitch (Robot Is Flat)", Math.abs(cache.pitch.getDegrees()) < 5.0);

    setBrakeMode(false);
    // MotorTestSparkMax.checkMotors(this,
    //   Arrays.asList(new MotorConfig<>("MasterR", masterR), new MotorConfig<>("SlaveR", slaveR)),
    //   new TestConfig().amps(5.0, 2.0).rpm(90.0, 200.0, encoderR::getVelocity));
    // MotorTestSparkMax.checkMotors(this,
    //   Arrays.asList(new MotorConfig<>("MasterL", masterL), new MotorConfig<>("SlaveL", slaveL)),
    //   new TestConfig().amps(5.0, 2.0).rpm(90.0, 200.0, encoderL::getVelocity));
  }
}
