package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Hood extends SmartSubsystem {
  public static class DataCache {
    public double amps;
  }
  private final DoubleSolenoid solenoid;
  
  // private final CANSparkMax motor;
  private final DataCache cache = new DataCache();

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
    if (extend) {
      solenoid.set(Value.kForward);
    }
    else {
      solenoid.set(Value.kReverse);
    }
  }
  @Override
  public void runTests() {
    // TODO
  }
}
