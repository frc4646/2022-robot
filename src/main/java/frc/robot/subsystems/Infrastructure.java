package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;

public class Infrastructure extends SmartSubsystem {
  public static class DataCache {
    public double battery;
  }

  private final Compressor compressor;
  private final PneumaticsControlModule pcm;
  private final UsbCamera camera;
  private final DataCache cache = new DataCache();

  public Infrastructure() {
    compressor = new Compressor(Constants.CAN.PNEUMATIC_CONTROL_MODULE, PneumaticsModuleType.CTREPCM);
    pcm = new PneumaticsControlModule(Constants.CAN.PNEUMATIC_CONTROL_MODULE);
    if (Constants.INFRASTRUCTURE.CAMERA_STREAM) {
      camera = CameraServer.startAutomaticCapture();
    }
    else {
      camera = null;
    }
  }

  @Override
  public void cacheSensors() {
    cache.battery = RobotController.getBatteryVoltage();
  }
  
  /** Enable/disable the {@link Compressor} closed loop, which <i>automatically</i> runs the {@link Compressor} when pressure is low */
  public void setCompressor(boolean enable) {
    if(enable) {
      compressor.enableDigital();
    } else {
      compressor.disable();
    }
  }
  
  @Override
  public void updateDashboard() {   
    SmartDashboard.putNumber("Battery", cache.battery);
  }

  @Override
  public void runTests() {
    Test.add(this, "Compressor - Connected", !pcm.getCompressorNotConnectedStickyFault());
    Test.add(this, "Compressor - Current Low", !pcm.getCompressorCurrentTooHighStickyFault());  // Max continuous 12V / 17A
    Test.add(this, "Compressor - Not Shorted", !pcm.getCompressorShortedStickyFault());
    Test.add(this, "Battery - Voltage", RobotController.getBatteryVoltage() > 13.0);
    if (Constants.INFRASTRUCTURE.CAMERA_STREAM) {
      Test.add(this, "Camera - Enabled", camera.isEnabled());
    }
  }
}
