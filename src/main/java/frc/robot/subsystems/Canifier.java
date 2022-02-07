package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Canifier extends SubsystemBase {
  public static class DataCache {
    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;
  }

  private final CANifier canifier;
  private DataCache cache = new DataCache();

  public Canifier() {
    canifier = new CANifier(Constants.CAN.CANIFIER);
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 10, Constants.CAN_TIMEOUT_LONG);
  }

  public void setLEDs(double red, double green, double blue) {
    if (red != cache.red || green != cache.green || blue != cache.blue) {
      cache.red = red;
      cache.green = green;
      cache.blue = blue;
      canifier.setLEDOutput(cache.red, CANifier.LEDChannel.LEDChannelA);
      canifier.setLEDOutput(cache.green, CANifier.LEDChannel.LEDChannelB);
      canifier.setLEDOutput(cache.blue, CANifier.LEDChannel.LEDChannelC);
    }
  }
}