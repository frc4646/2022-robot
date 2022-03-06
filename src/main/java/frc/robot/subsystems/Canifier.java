package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.Test;

public class Canifier extends SmartSubsystem {
  public static class DataCache {
    double red = 0.0, green = 0.0, blue = 0.0;
    boolean isTurrentHome;
  }

  public static class COLOR {
    double red = 0.0, green = 0.0, blue = 0.0;
    public COLOR(double red, double green, double blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
  }

  private final CANifier canifier;
  private DataCache cache = new DataCache();

  public Canifier() {
    canifier = new CANifier(Constants.CAN.CANIFIER);
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
  public void updateDashboard() {
    SmartDashboard.putBoolean("Canifier: Turret Home", cache.isTurrentHome);
  }

  public void setLEDs(double red, double green, double blue) {
    if (red == cache.red && green == cache.green && blue == cache.blue) {
      return;
    }
    cache.red = red;
    cache.green = green;
    cache.blue = blue;
    canifier.setLEDOutput(cache.red, CANifier.LEDChannel.LEDChannelA);
    canifier.setLEDOutput(cache.green, CANifier.LEDChannel.LEDChannelB);
    canifier.setLEDOutput(cache.blue, CANifier.LEDChannel.LEDChannelC);
  }
  
  public void setLEDs(COLOR color) {
    setLEDs(color.red, color.green, color.blue);
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
