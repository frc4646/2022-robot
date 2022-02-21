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
    masterL.getPIDController().setP(Constants.Drivetrain.P_LEFT);
    masterL.getPIDController().setI(Constants.Drivetrain.I_LEFT);
    masterL.getPIDController().setD(Constants.Drivetrain.D_LEFT);
    masterL.getPIDController().setFF(Constants.Drivetrain.F_LEFT);
    masterR.getPIDController().setP(Constants.Drivetrain.P_RIGHT);
    masterR.getPIDController().setI(Constants.Drivetrain.I_RIGHT);
    masterR.getPIDController().setD(Constants.Drivetrain.D_RIGHT);
    masterR.getPIDController().setFF(Constants.Drivetrain.F_RIGHT);

    isBrakeMode = true;
    setBrakeMode(false);

    resetEncoders();
    gyro.reset();
    odometry = new DifferentialDriveOdometry(cache.heading);
    
    // ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    // dashDistanceL = DashboardControls.getGraph(tab, "Distance L", 0.0).getEntry();
    // dashDistanceR = DashboardControls.getGraph(tab, "Distance R", 0.0).getEntry();
    // dashRPML = DashboardControls.getGraph(tab, "RPM L", 0.0).getEntry();
    // dashRPMR = DashboardControls.getGraph(tab, "RPM R", 0.0).getEntry();
    // dashHeading = DashboardControls.getGraph(tab, "Heading", 0.0).getEntry();
    // dashPitch = DashboardControls.getGraph(tab, "Pitch", 0.0).getEntry();
  }

  public void configureMotor(CANSparkMax motor, boolean isLeft, boolean isMaster) {
    motor.setInverted(!isLeft);
    motor.enableVoltageCompensation(Constants.Drivetrain.VOLTAGE_COMPENSATION);
    motor.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT); // TODO find more examples to confirm what values are best
  }

  @Override
  public void cacheSensors() {
    cache.distanceL = masterL.getEncoder().getPosition();
    cache.distanceR = masterR.getEncoder().getPosition();
    cache.rpmL = masterL.getEncoder().getVelocity();
    cache.rpmR = masterR.getEncoder().getVelocity();
    cache.heading = Rotation2d.fromDegrees(gyro.getFusedHeading()).rotateBy(gyroOffset);
    cache.pitch = Rotation2d.fromDegrees(gyro.getPitch());
    double metersL = 0.0;  // TODO m_leftEncoder.getDistance()
    double metersR = 0.0;  // TODO m_rightEncoder.getDistance()
    odometry.update(getHeading(), metersL, metersR);
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

  public void resetPose(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
    // TODO update cache or something else about gyro?
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
    // TODO is it this? masterL.getPIDController().setReference(left, ControlType.kVoltage);
    // TODO is it this? masterR.getPIDController().setReference(right, ControlType.kVoltage);
  }

  public void setClosedLoopVelocity(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedforward = Constants.Drivetrain.FEED_FORWARD.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = Constants.Drivetrain.FEED_FORWARD.calculate(speeds.rightMetersPerSecond);
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

  public Pose2d getPose() {
    return null;  // TODO
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return null;  // TODO
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
