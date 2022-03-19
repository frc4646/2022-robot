package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.team254.drivers.SparkMaxFactory;
import frc.team4646.Test;

public class Agitator extends SmartSubsystem {
  private class DataCache {
    public boolean inBrakeMode = true;
  }
  private class OutputCache {
    public double setpointL = 0.0, setpointR = 0.0;
  }
  private final CANSparkMax masterL, masterR;
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();

  public Agitator() {
    masterL = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.AGITATOR_L, true);
    masterR = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.AGITATOR_R, false);
    configureMotor(masterL);
    configureMotor(masterR);
    updateBrakeMode(false);
  }

  protected void configureMotor(CANSparkMax motor) {
    motor.enableVoltageCompensation(12.0);
    motor.setOpenLoopRampRate(Constants.AGITATOR.OPEN_LOOP_RAMP);
  }

  @Override
  public void updateHardware() {
    updateMotors();
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    updateBrakeMode(true);
  }

  @Override
  public void onDisable() {
    updateBrakeMode(false);
  }

  public void setOpenLoop(double left, double right) {
    outputs.setpointL = left;
    outputs.setpointR = right;
  }

  private void updateMotors() {
    masterL.set(outputs.setpointL);
    masterR.set(outputs.setpointR);
  }

  private void updateBrakeMode(boolean enable) {
    if (enable != cache.inBrakeMode) {
      cache.inBrakeMode = enable;
      IdleMode mode = cache.inBrakeMode ? IdleMode.kBrake : IdleMode.kCoast;
      masterL.setIdleMode(mode);
      masterR.setIdleMode(mode);
    }
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, masterR);
    Test.add(this, "Brake Mode", cache.inBrakeMode);
  }
}
