package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.util.Test;
import frc.team254.drivers.SparkMaxFactory;

public class Agitator extends SmartSubsystem {
  private final CANSparkMax masterL, slaveR;

  public Agitator() {
    masterL = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.AGITATOR_L, true);
    masterL.setIdleMode(IdleMode.kBrake);
    masterL.enableVoltageCompensation(12.0);
    masterL.setOpenLoopRampRate(Constants.AGITATOR.OPEN_LOOP_RAMP);

    slaveR = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.CAN.AGITATOR_R, masterL, true);
    slaveR.setIdleMode(IdleMode.kBrake);
    slaveR.enableVoltageCompensation(12.0);
    slaveR.setOpenLoopRampRate(Constants.AGITATOR.OPEN_LOOP_RAMP);    
  }

  public void setOpenLoop(double percent) {
    masterL.set(percent);
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, masterL);
    Test.checkFirmware(this, slaveR);    
  }
}
