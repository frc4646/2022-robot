package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends SmartSubsystem {
  private final CANSparkMax leftMaster, rightSlave;

  public Shooter() {
    leftMaster = new CANSparkMax(Constants.Ports.SHOOTER_L, MotorType.kBrushless);
    rightSlave = new CANSparkMax(Constants.Ports.SHOOTER_R, MotorType.kBrushless);
   // leftMaster.setInverted(true);
    rightSlave.follow(leftMaster);
    // TODO doesn't slave need to be inverted?
    leftMaster.setIdleMode(IdleMode.kCoast);
    rightSlave.setIdleMode(IdleMode.kCoast);
    leftMaster.enableVoltageCompensation(12.0);
    rightSlave.enableVoltageCompensation(12.0);
    // leftMotor.setSmartCurrentLimit(40);
    // rightMotor.setSmartCurrentLimit(40);

    // TODO limit supply current
    // TODO PIDF
    // TODO peak output forward direction only
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Shooter/Velocity", leftMaster.getEncoder().getVelocity());
  }

  public void setOpenLoop(double percent) {
    leftMaster.set(-percent);
  }

  public void setClosedLoopVelocity(double rpm) {
    // this is closed loop
    // TODO set velocity control mode
  }
  public int getVelocityRPM() {
    return 0;  // TODO read sensor
  }

  public boolean isOnTarget() {
    // TODO is close enough
    // TODO increment how many times is stable    
    return false;  // TODO decide is enough times is stable
  }
}
