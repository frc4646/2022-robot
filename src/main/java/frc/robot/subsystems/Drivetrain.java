package frc.robot.subsystems;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.Drivetrain.FEED_FORWARD_GAIN_STATIC, Constants.Drivetrain.FEED_FORWARD_GAIN_VELOCITY, Constants.Drivetrain.FEED_FORWARD_GAIN_ACCEL);
  private DataCache cache = new DataCache();
  // private final NetworkTableEntry dashDistanceL, dashDistanceR, dashRPML, dashRPMR, dashHeading, dashPitch;
  
  private boolean isBrakeMode;
  private Rotation2d gyroOffset = Rotation2d.fromDegrees(0.0);

  public Drivetrain() {
    masterR = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.DRIVETRAIN_FL);
    slaveR = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.CAN.DRIVETRAIN_BL, masterR, false);
    masterL = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.DRIVETRAIN_FR, true);
    slaveL = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.CAN.DRIVETRAIN_BR, masterL, false);
    gyro = new AHRS();

    configureMotor(masterL, true, true);
    configureMotor(slaveL, true, false);
    configureMotor(masterR, false, true);
    configureMotor(slaveR, false, false);

    isBrakeMode = true;
    setBrakeMode(false);

    resetEncoders();
    gyro.reset();
    odometry = new DifferentialDriveOdometry(cache.heading);
    
    // ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    // dashDistanceL = tab.add("Distance L", cache.distanceL).withWidget(BuiltInWidgets.kGraph).getEntry();
    // dashDistanceR = tab.add("Distance R", cache.distanceR).withWidget(BuiltInWidgets.kGraph).getEntry();
    // dashRPML = tab.add("RPM L", cache.rpmL).withWidget(BuiltInWidgets.kGraph).getEntry();
    // dashRPMR = tab.add("RPM R", cache.rpmR).withWidget(BuiltInWidgets.kGraph).getEntry();
    // dashHeading = tab.add("Heading", cache.heading.getDegrees()).withWidget(BuiltInWidgets.kGraph).getEntry();
    // dashPitch = tab.add("Pitch", cache.pitch.getDegrees()).withWidget(BuiltInWidgets.kGraph).getEntry();
  }

  public void configureMotor(CANSparkMax motor, boolean isLeft, boolean isMaster) {
    motor.setInverted(!isLeft);
    motor.enableVoltageCompensation(Constants.Drivetrain.VOLTAGE_COMPENSATION);
    motor.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT); // TODO find more examples to confirm what values are best
    if (isMaster) {
      motor.getPIDController().setP(Constants.Drivetrain.P);
      motor.getPIDController().setI(Constants.Drivetrain.I);
      motor.getPIDController().setD(Constants.Drivetrain.D);
      motor.getPIDController().setFF(Constants.Drivetrain.F);
    }
  }

  @Override
  public void cacheSensors() {
    cache.distanceL = masterL.getEncoder().getPosition();
    cache.distanceR = masterR.getEncoder().getPosition();
    cache.rpmL = masterL.getEncoder().getVelocity();
    cache.rpmR = masterR.getEncoder().getVelocity();
    cache.heading = Rotation2d.fromDegrees(gyro.getFusedHeading()).rotateBy(gyroOffset);
    cache.pitch = Rotation2d.fromDegrees(gyro.getPitch());
  }

  @Override
  public void updateDashboard() {
    // dashDistanceL.setDouble(cache.distanceL);
    // dashDistanceR.setDouble(cache.distanceR);
    // dashRPML.setDouble(cache.rpmL);
    // dashRPMR.setDouble(cache.rpmR);
    // dashHeading.setDouble(cache.heading.getDegrees());
    // dashPitch.setDouble(cache.pitch.getDegrees());

    SmartDashboard.putNumber("Drive: Heading", cache.heading.getDegrees());
    SmartDashboard.putNumber("Drive: Pitch", cache.pitch.getDegrees());
    if (Constants.Drivetrain.TUNING) {
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

  public void setClosedLoopVelocity(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedforward = feedForward.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = feedForward.calculate(speeds.rightMetersPerSecond);
    // TODO leftMaster.getPIDController().setReference(value, ctrl, pidSlot, arbFeedforward)
    // TODO rightMaster.getPIDController().setReference(value, ctrl, pidSlot, arbFeedforward)
  }

  public void setHeading(Rotation2d heading) {
    // gyroOffset = heading.rotateBy(Rotation2d.fromDegrees(gyro.getFusedHeading()).rotateBy(Rotation2d.fromDegrees(180.0)));
    cache.heading = heading;
  }

  public double getDistance() {
    return (cache.distanceL + cache.distanceR) / 2.0;
  }

  public double getRPM(){
    return (cache.rpmL + cache.rpmR) / 2.0;
  }

  public Rotation2d getHeading() {
    return cache.heading;
  }

  public Rotation2d getPitch() {
    return cache.pitch;
  }

  public void updateOdometry() {
    // TODO m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
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
      Arrays.asList(new MotorConfig<>("MasterL", masterL), new MotorConfig<>("SlaveL", slaveL)),
      new TestConfig().amps(5.0, 2.0).rpm(90.0, 200.0, masterL.getEncoder()::getVelocity));
    MotorTestSparkMax.checkMotors(this,
      Arrays.asList(new MotorConfig<>("MasterR", masterR), new MotorConfig<>("SlaveR", slaveR)),
      new TestConfig().amps(5.0, 2.0).rpm(90.0, 200.0, masterR.getEncoder()::getVelocity));
  }
}
