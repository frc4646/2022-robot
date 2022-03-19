package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team4646.LEDColor;
import frc.team4646.Test;

public class Canifier extends SmartSubsystem {
  private class DataCache {
    double red = 0.0, green = 0.0, blue = 0.0;
    boolean isTurrentHome;
  }

  private final CANifier canifier;
  private final DataCache cache = new DataCache();

  public Canifier() {
    canifier = new CANifier(Constants.CAN.CANIFIER);
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 255, Constants.CAN_TIMEOUT);
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
  public void updateDashboard(boolean showDetails) {
    SmartDashboard.putBoolean("Canifier: Turret Home", cache.isTurrentHome);
  }
  
  public void setLEDs(LEDColor color) {
    if (color.red == cache.red && color.green == cache.green && color.blue == cache.blue) {
      return;
    }
    cache.red = color.red;
    cache.green = color.green;
    cache.blue = color.blue;
    canifier.setLEDOutput(cache.red, CANifier.LEDChannel.LEDChannelA);
    canifier.setLEDOutput(cache.green, CANifier.LEDChannel.LEDChannelB);
    canifier.setLEDOutput(cache.blue, CANifier.LEDChannel.LEDChannelC);
  }

  public boolean isTurretHome() {
    return cache.isTurrentHome;
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, canifier);
    Test.add(this, "Turret Home", cache.isTurrentHome);
  }
}
