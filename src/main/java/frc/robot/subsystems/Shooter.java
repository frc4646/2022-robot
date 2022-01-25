package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final CANSparkMax leftMotor, rightMotor;

  public Shooter() {
    leftMotor = new CANSparkMax(Constants.Ports.SHOOTER_L, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.Ports.SHOOTER_R, MotorType.kBrushless);
    
    IdleMode mode = IdleMode.kCoast;
    leftMotor.setIdleMode(mode);
    rightMotor.setIdleMode(mode);
    leftMotor.enableVoltageCompensation(12.0);
    rightMotor.enableVoltageCompensation(12.0);
    // leftMotor.follow(rightMotor);
    // leftMotor.setSmartCurrentLimit(40);
    // rightMotor.setSmartCurrentLimit(40);

    // TODO we have 2 motors So set one as a follower but inverted
    // TODO limit supply current
    // TODO PIDF
    // TODO peak output forward direction only
    // TODO coast mode

  }

  public void setOpenLoop(double percent) {
    leftMotor.set(percent);
    rightMotor.set(percent);
  }

  public void setSpeed(double velocity) {
    // this is closed loop
    // TODO set velocity control mode
  }
  public int getSpeed() {
    return 0;  // TODO read sensor
  }

  public boolean isOnTarget() {
    // TODO is close enough
    // TODO increment how many times is stable    
    return false;  // TODO decide is enough times is stable
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Velocity", leftMotor.getEncoder().getVelocity());
  }
}
