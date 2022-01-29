package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends SmartSubsystem {
  public static class DataCache {
    public double rpm;
  }

  private final CANSparkMax leftMaster, rightSlave;
  private final DataCache cache = new DataCache();

  private double closedLoopTargetRPM = 0.0;

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
  public void cacheSensors() {
    cache.rpm = leftMaster.getEncoder().getVelocity();
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Shooter/Velocity", cache.rpm);
  }

  public void setOpenLoop(double percent) {
    leftMaster.set(-percent);  // TODO make inverted instead, otherwise close loop will be wrong
  }

  public void setClosedLoopVelocity(double rpm) {
    closedLoopTargetRPM = rpm;
    // TODO set motor to velocity control mode
  }
  public double getVelocityRPM() {
    return cache.rpm;
  }

  public boolean isOnTarget() {
    // TODO increment how many times is stable    
    return Math.abs(closedLoopTargetRPM - getVelocityRPM()) > Constants.Shooter.ERROR_ALLOWED_RPM;
  }
}
