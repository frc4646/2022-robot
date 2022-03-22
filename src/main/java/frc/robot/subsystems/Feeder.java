package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.SparkMaxFactory;
import frc.team4646.Test;

public class Feeder extends SmartSubsystem {
  private class DataCache {
    public boolean breakBeamShooter = false, breakBeamHopper = false, breakBeamIndexer = false;
    public boolean inBrakeMode = false;
  }
  private class OutputCache {
    public double setpoint = 0.0;
  }

  private final CANSparkMax motor;
  private final DigitalInput breakBeam, breakBeamIndexer, breakBeamHopper;
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();

  public Feeder() {
    motor = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.FEEDER);
    breakBeam = new DigitalInput(Constants.DIGITAL.BREAK_BEAM_SHOOTER);
    breakBeamIndexer = new DigitalInput(Constants.DIGITAL.BREAK_BEAM_INDEXER);
    breakBeamHopper = new DigitalInput(Constants.DIGITAL.BREAK_BEAM_HOPPER);

    motor.setInverted(true);
    motor.enableVoltageCompensation(12.0);
    motor.setOpenLoopRampRate(Constants.FEEDER.OPEN_LOOP_RAMP);
    setBrakeMode(!cache.inBrakeMode);
  }

  @Override
  public void cacheSensors () {
    cache.breakBeamShooter = !breakBeam.get();
    cache.breakBeamHopper = !breakBeamHopper.get();
    cache.breakBeamIndexer = !breakBeamIndexer.get();
  }

  @Override
  public void updateHardware() {
    updateMotors();
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    SmartDashboard.putBoolean("Feeder: Shooter Cargo", isShooterLoaded());
    SmartDashboard.putBoolean("Feeder: Hopper Cargo", isHooperFull());
    SmartDashboard.putBoolean("Feeder: Indexed Cargo", isCargoIndexed());
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    setBrakeMode(true);
  }

  @Override
  public void onDisable() {
    setBrakeMode(false);
  }

  public void setOpenLoop(double percent) { outputs.setpoint = percent; }
  public boolean isHooperFull() { return cache.breakBeamHopper; }
  public boolean isCargoIndexed() { return cache.breakBeamIndexer; }
  public boolean isShooterLoaded() { return cache.breakBeamShooter; }

  private void updateMotors() {
    motor.set(outputs.setpoint);
  }

  private void setBrakeMode(boolean enable) {
    if (cache.inBrakeMode != enable) {
      IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
      motor.setIdleMode(mode);
      cache.inBrakeMode = enable;
    }
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, motor);
  }
}
