package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.team254.drivers.SparkMaxFactory;
import frc.team4646.Test;

public class Agitator extends SmartSubsystem {
  private final CANSparkMax masterL, masterR;
  private boolean isBrakeMode;

  public Agitator() {
    masterL = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.AGITATOR_L, true);
    masterR = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.AGITATOR_R, false);
    configureMotor(masterL);
    configureMotor(masterR);

    isBrakeMode = true;
    setBrakeMode(false);
  }

  protected void configureMotor(CANSparkMax motor) {
    motor.enableVoltageCompensation(12.0);
    motor.setOpenLoopRampRate(Constants.AGITATOR.OPEN_LOOP_RAMP);
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    setBrakeMode(true);
  }

  @Override
  public void onDisable() {
    setBrakeMode(false);
  }

  public void setOpenLoop(double left, double right) {
    masterL.set(left);
    masterR.set(right);
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode == enable) {
      return;
    }
    IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
    masterL.setIdleMode(mode);
    masterR.setIdleMode(mode);
    isBrakeMode = enable;
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, masterR);  
  }
}
