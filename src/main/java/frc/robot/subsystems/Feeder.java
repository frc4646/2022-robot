package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;
import frc.team254.drivers.SparkMaxFactory;

public class Feeder extends SmartSubsystem {
  private static class DataCache {
    public boolean shooterLoaded;
  }

  private final CANSparkMax motor;
  private final DigitalInput breakBeam;
  private DataCache cache = new DataCache();

  public Feeder() {
    motor = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.FEEDER);
    breakBeam = new DigitalInput(Constants.Digital.FEEDER_BREAK_BEAM);

    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kBrake);
    motor.enableVoltageCompensation(12.0);
    motor.setOpenLoopRampRate(Constants.Feeder.OPEN_LOOP_RAMP);
    // TODO supply current limiting
  }

  @Override
  public void cacheSensors () {
    cache.shooterLoaded = !breakBeam.get();
  }

  @Override
  public void updateDashboard() {    
    SmartDashboard.putBoolean("Feeder: Loaded", isShooterLoaded());
  }

  public void setOpenLoop(double percent) {
    motor.set(percent);
  }

  public boolean isShooterLoaded() {
    return cache.shooterLoaded;
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, motor);
  }
}
