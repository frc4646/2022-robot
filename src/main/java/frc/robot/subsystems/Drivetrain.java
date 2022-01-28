package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;

public class Drivetrain extends SmartSubsystem {
  private final CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;
  // private final PigeonIMU gyro;
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.Drivetrain.FEED_FORWARD_GAIN_STATIC, Constants.Drivetrain.FEED_FORWARD_GAIN_VELOCITY, Constants.Drivetrain.FEED_FORWARD_GAIN_ACCEL);
  
  private boolean isBrakeMode;

  public Drivetrain() {
    leftMaster = new CANSparkMax(Constants.Ports.DRIVETRAIN_FL, MotorType.kBrushless);
    leftSlave = new CANSparkMax(Constants.Ports.DRIVETRAIN_BL, MotorType.kBrushless);
    rightMaster = new CANSparkMax(Constants.Ports.DRIVETRAIN_FR, MotorType.kBrushless);
    rightSlave = new CANSparkMax(Constants.Ports.DRIVETRAIN_BR, MotorType.kBrushless);
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
  }

  public void configureMotor(CANSparkMax motor, boolean isLeft) {
    motor.setInverted(!isLeft);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT); // TODO find more examples to confirm what values are best
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
    return null;
    //TODO return Rotation2d.fromDegrees(gyro.getFusedHeading()).rotateBy(mGyroOffset);
  }

  public void updateOdometry() {
    // TODO m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
}
