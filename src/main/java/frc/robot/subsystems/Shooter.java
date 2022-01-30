package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends SmartSubsystem {
  public static class DataCache {
    public double rpmL;
    public double rpmR;
  }

  private final CANSparkMax leftMaster, rightSlave;
  private final DataCache cache = new DataCache();

  private double targetVelocityRPM = Double.POSITIVE_INFINITY;
  private double demand = 0.0;
  private int stableCounts = 0;

  public Shooter() {
    leftMaster = new CANSparkMax(Constants.Ports.SHOOTER_L, MotorType.kBrushless);
    rightSlave = new CANSparkMax(Constants.Ports.SHOOTER_R, MotorType.kBrushless);

    rightSlave.follow(leftMaster, true);
    leftMaster.setInverted(true);
    leftMaster.setIdleMode(IdleMode.kCoast);
    rightSlave.setIdleMode(IdleMode.kCoast);
    leftMaster.enableVoltageCompensation(12.0);
    rightSlave.enableVoltageCompensation(12.0);
    // leftMotor.setSmartCurrentLimit(40);
    // rightMotor.setSmartCurrentLimit(40);
    //leftMaster.getPIDController().setFF(4900);

    // TODO limit supply current
    // TODO PIDF
    // TODO peak output forward direction only
  }

  @Override
  public void cacheSensors() {
    cache.rpmL = leftMaster.getEncoder().getVelocity();
    cache.rpmR = rightSlave.getEncoder().getVelocity();
    stableCounts++;
    if (!isOnTarget()) {
      stableCounts = 0;
    }
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Shooter RPM L", cache.rpmL);
    SmartDashboard.putNumber("Shooter RPM R", cache.rpmR);
    SmartDashboard.putNumber("Shooter RPM", getVelocityRPM());
    SmartDashboard.putNumber("Shooter Demand", demand);
    SmartDashboard.putNumber("Shooter Amps L", leftMaster.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Amps R", rightSlave.getOutputCurrent());
  }

  public void setOpenLoop(double percent) {
    leftMaster.set(percent);  // TODO make inverted instead, otherwise close loop will be wrong
    demand = percent;
  }

  public void setClosedLoopVelocity(double rpm) {
    setTargetVelocity(rpm);
    leftMaster.getPIDController().setReference(rpm, ControlType.kVelocity);
    demand = rpm;
  }

  public void setTargetVelocity(double rpm) {
    targetVelocityRPM = rpm;
  }

  public double getVelocityRPM() {
    return (cache.rpmL + cache.rpmR) / 2.0;
  }

  public boolean isOnTarget() {
    return Math.abs(targetVelocityRPM - getVelocityRPM()) < Constants.Shooter.ERROR_ALLOWED_RPM;
  }

  public boolean isStable() {
    return stableCounts > 5;
  }
}
