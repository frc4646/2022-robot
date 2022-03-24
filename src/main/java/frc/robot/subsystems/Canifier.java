package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierFaults;
import com.ctre.phoenix.CANifierStatusFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team4646.LEDColor;
import frc.team4646.Test;

public class Canifier extends SmartSubsystem {
  private class DataCache {
    public LEDColor color = new LEDColor();
    public boolean isTurrentHome;
  }
  private class OutputCache {
    public LEDColor color = new LEDColor();
  }

  private final CANifier canifier;
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();

  public Canifier() {
    canifier = new CANifier(Constants.CAN.CANIFIER);
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 50, Constants.CAN_TIMEOUT);
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 10, Constants.CAN_TIMEOUT);
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 1000, Constants.CAN_TIMEOUT);
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 1000, Constants.CAN_TIMEOUT);
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_5_PwmInputs2, 1000, Constants.CAN_TIMEOUT);
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 1000, Constants.CAN_TIMEOUT);
  }

  @Override
  public void cacheSensors() {
    cache.isTurrentHome = !canifier.getGeneralInput(CANifier.GeneralPin.LIMF);
  }

  @Override
  public void updateHardware() {
    updateLEDs();
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    SmartDashboard.putBoolean("Canifier: Turret Home", cache.isTurrentHome);
  }
  
  public void setLEDs(LEDColor color) { outputs.color = color; }
  public boolean isTurretHome() { return cache.isTurrentHome; }

  private void updateLEDs() {
    if (!cache.color.isEqual(outputs.color)) {      
      cache.color = outputs.color;
      cache.color = new LEDColor(255, 0, 0);  // TODO REMOVE ME
      canifier.setLEDOutput(cache.color.red / 255.0, CANifier.LEDChannel.LEDChannelB);
      canifier.setLEDOutput(cache.color.green / 255.0, CANifier.LEDChannel.LEDChannelA);
      canifier.setLEDOutput(cache.color.blue / 255.0, CANifier.LEDChannel.LEDChannelC);
    }
  }

  @Override
  public void runTests() {
    CANifierFaults faults = new CANifierFaults();
    canifier.getFaults(faults);

    Test.checkFirmware(this, canifier);
    Test.add(this, "Turret Home", cache.isTurrentHome);
    Test.add(this, String.format("Faults: 0x%X", faults.toBitfield()), faults.hasAnyFault());
  }
}
