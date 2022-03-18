package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.SparkMaxFactory;
import frc.team4646.Test;

public class Feeder extends SmartSubsystem {
  private class DataCache {
    public boolean shooterLoaded, shooterLoadedBottom;
    public double position;
  }

  private final CANSparkMax motor;
  private final DigitalInput breakBeam, breakBeamBottom;
  private final DataCache cache = new DataCache();
  private boolean isBrakeMode = true;

  public Feeder() {
    motor = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.FEEDER);
    breakBeam = new DigitalInput(Constants.DIGITAL.FEEDER_BREAK_BEAM);
    breakBeamBottom = new DigitalInput(Constants.DIGITAL.FEEDER_BOTTOM_BREAK_BEAM);

    motor.setInverted(true);
    motor.enableVoltageCompensation(12.0);
    motor.setOpenLoopRampRate(Constants.FEEDER.OPEN_LOOP_RAMP);

    setBrakeMode(!isBrakeMode);
  }

  @Override
  public void cacheSensors () {
    cache.shooterLoaded = !breakBeam.get();
    cache.shooterLoadedBottom = !breakBeamBottom.get();
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    SmartDashboard.putBoolean("Feeder: Loaded", isShooterLoaded());
    SmartDashboard.putBoolean("Feeder: Bottom Loaded", isShooterLoadedBottom());
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    setBrakeMode(true);
  }

  @Override
  public void onDisable() {
    setBrakeMode(false);
  }

  public void setOpenLoop(double percent) {
    motor.set(percent);
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode == enable) {
      return;
    }
    IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
    motor.setIdleMode(mode);
    isBrakeMode = enable;
  }

  public double getPosition() { return cache.position / Constants.FEEDER.GEAR_RATIO; }

  public int getQueuedCargo() {
    return (isShooterLoaded() ? 1 : 0) +
           (isShooterLoadedBottom() ? 1 : 0);  // TODO
  }

  public int getQueuedCargoCorrect() {
    return 0;  // TODO
  }

  public int getQueuedCargoWrong() {
    return 0;  // TODO
  }

  public boolean isShooterLoaded() { return cache.shooterLoaded; }
  public boolean isShooterLoadedBottom() { return cache.shooterLoadedBottom; }

  @Override
  public void runTests() {
    Test.checkFirmware(this, motor);
  }
}
