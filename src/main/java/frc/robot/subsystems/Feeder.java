package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Digital;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Feeder extends SmartSubsystem {
  public static class DataCache {
    public boolean hasBall;
  }

  private final VictorSPX motor;
  private final DigitalInput breakBeam;
  private DataCache cache = new DataCache();

  public Feeder() {
    motor = new VictorSPX(Constants.Ports.FEEDER);
    breakBeam = new DigitalInput(Constants.Digital.FEEDER_BREAK_BEAM);
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
}
