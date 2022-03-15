package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.team254.drivers.SparkMaxFactory;
import frc.team4646.Test;

public class Agitator extends SmartSubsystem {
  private final CANSparkMax masterL, masterR;

  public Agitator() {
    masterL = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.AGITATOR_L, true);
    masterL.setIdleMode(IdleMode.kBrake);
    masterL.enableVoltageCompensation(12.0);
    masterL.setOpenLoopRampRate(Constants.AGITATOR.OPEN_LOOP_RAMP);

    masterR = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.AGITATOR_R, false);
    masterR.setIdleMode(IdleMode.kBrake);
    masterR.enableVoltageCompensation(12.0);
    masterR.setOpenLoopRampRate(Constants.AGITATOR.OPEN_LOOP_RAMP);    
  }

  public void setOpenLoop(double left, double right) {
    masterL.set(left);
    masterR.set(right);
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, masterR);  
  }
}
