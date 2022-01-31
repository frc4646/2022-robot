package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Turret extends SmartSubsystem {
  public static class DataCache {
    public double amps;
  }

  private final CANSparkMax motor;
  private final DataCache cache = new DataCache();

  public Turret() {
    motor = new CANSparkMax(Constants.Ports.TURRET, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void cacheSensors() {
    cache.amps = motor.getOutputCurrent();
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber(this.getName() + " Amps", cache.amps);
  }

  public void setOpenLoop(double percent) {
    motor.set(percent);
  }
}
