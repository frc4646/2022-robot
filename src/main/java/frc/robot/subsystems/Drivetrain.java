package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;
  
  private boolean isBrakeMode;

  public Drivetrain() {
    leftMaster = new CANSparkMax(Constants.Ports.DRIVETRAIN_FL, MotorType.kBrushless);
    leftSlave = new CANSparkMax(Constants.Ports.DRIVETRAIN_BL, MotorType.kBrushless);
    rightMaster = new CANSparkMax(Constants.Ports.DRIVETRAIN_FR, MotorType.kBrushless);
    rightSlave = new CANSparkMax(Constants.Ports.DRIVETRAIN_BR, MotorType.kBrushless);
    
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    configureMotor(leftMaster, true);
    configureMotor(leftSlave, true);
    configureMotor(rightMaster, false);
    configureMotor(rightSlave, false);

    isBrakeMode = true;
    setBrakeMode(false);
  }

  public void configureMotor(CANSparkMax motor, boolean isLeft) {
    motor.setInverted(!isLeft);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT); // TODO find more examples to confirm what values are best
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode != enable)
    {
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

  public Rotation2d getHeading() {
    return null;  // TODO
  }
}
