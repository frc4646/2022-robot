package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Feeder extends SmartSubsystem {
  private static class DataCache {
    public boolean hasBall;
  }

  private final VictorSPX motor;
  private final DigitalInput breakBeam;
  private DataCache cache = new DataCache();

  public Feeder() {
    motor = new VictorSPX(Constants.CAN.FEEDER);
    breakBeam = new DigitalInput(Constants.Digital.FEEDER_BREAK_BEAM);

    motor.setNeutralMode(NeutralMode.Brake);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(.25);
    // TODO supply current limiting
    // TODO add sensor for if ball is in indexer
  }

  @Override
  public void cacheSensors () {
    cache.hasBall = breakBeam.get();
  }

  @Override
  public void updateDashboard() {    
    SmartDashboard.putBoolean("Feeder Ball", isBallPresent());
  }

  public void setOpenLoop(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }

  public boolean isBallPresent() {
    return cache.hasBall;
  } 

  @Override
  public void runTests() {
    Test.checkFirmware(new Test.FirmwareTalon(this, motor));
  }
}
