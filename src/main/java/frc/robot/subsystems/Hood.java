package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;

public class Hood extends SmartSubsystem {
  public static class DataCache {
    public double amps;
  }

  // private final CANSparkMax motor;
  private final DoubleSolenoid solenoid;
  private final DataCache cache = new DataCache();

  private boolean extended = false;

  public Hood() {
    solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.CLIMBER_L_OUT, Constants.Solenoid.CLIMBER_L_IN);
    // motor = new CANSparkMax(Constants.CAN.TURRET, MotorType.kBrushless);
    // motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void cacheSensors() {
    // cache.amps = motor.getOutputCurrent();
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber(this.getName() + " Amps", cache.amps);
  }

  public void setOpenLoop(double percent) {
    // motor.set(percent);
  }

  public void setExtend (boolean extend) {
    Value direction = (extend) ? Value.kForward : Value.kReverse;
    solenoid.set(direction);
    extended = extend;
  }

  public boolean isExtended() {
    return extended;
  }

  @Override
  public void runTests() {
    Test.checkSolenoid(this, solenoid);
  }
}
