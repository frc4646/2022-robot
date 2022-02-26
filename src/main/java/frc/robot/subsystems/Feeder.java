package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;
import frc.team254.drivers.SparkMaxFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Feeder extends SmartSubsystem {
  private static class DataCache {
    public boolean hasCargo;
  }

  private final VictorSPX motor;
  // private final CANSparkMax motor;
  private final DigitalInput breakBeam;
  private DataCache cache = new DataCache();

  public Feeder() {
    motor = new VictorSPX(Constants.CAN.FEEDER);
    // motor = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.FEEDER);
    breakBeam = new DigitalInput(Constants.Digital.FEEDER_BREAK_BEAM);

    motor.setNeutralMode(NeutralMode.Brake);
    motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(Constants.Feeder.OPEN_LOOP_RAMP);
    // motor.setIdleMode(IdleMode.kBrake);
    // motor.enableVoltageCompensation(12.0);
    // motor.setOpenLoopRampRate(Constants.Feeder.OPEN_LOOP_RAMP);
    // TODO supply current limiting
    // TODO add sensor for if ball is in indexer
  }

  @Override
  public void cacheSensors () {
    cache.hasCargo = !breakBeam.get();
    // cache.hasBall = false;  // TODO break beam currently not working
  }

  @Override
  public void updateDashboard() {    
    SmartDashboard.putBoolean("Feeder: Cargo", isCargoPresent());
  }

  public void setOpenLoop(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
    // motor.set(percent);
  }

  public boolean isCargoPresent() {
    return cache.hasCargo;
  } 

  @Override
  public void runTests() {
    Test.checkFirmware(this, motor);
  }
}
