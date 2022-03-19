package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team4646.Test;

public class Infrastructure extends SmartSubsystem {
  private class DataCache {
    public boolean enableCompressor = true;
    public double battery;
  }
  private class OutputCache {
    public boolean enableCompressor = true;
  }

  private final Compressor compressor;
  private final PneumaticsControlModule pcm;
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();

  public Infrastructure() {
    compressor = new Compressor(Constants.CAN.PNEUMATIC_CONTROL_MODULE, PneumaticsModuleType.CTREPCM);
    pcm = new PneumaticsControlModule(Constants.CAN.PNEUMATIC_CONTROL_MODULE);
    if (Constants.INFRASTRUCTURE.CAMERA_STREAM) {
      new Thread(() -> {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(416, 240);
        camera.setFPS(15);
      }).start();
    }
  }

  @Override
  public void cacheSensors() {
    cache.battery = RobotController.getBatteryVoltage();
  }

  @Override
  public void updateHardware() {
    updateCompressor();
  }
  
  @Override
  public void updateDashboard(boolean showDetails) {
    if (showDetails) {
      SmartDashboard.putNumber("Battery", cache.battery);
    }
  }
  
  public void setCompressor(boolean enable) {
    outputs.enableCompressor = enable;
  }

  private void updateCompressor() {
    if (outputs.enableCompressor != cache.enableCompressor) {
      cache.enableCompressor = outputs.enableCompressor;
      if(cache.enableCompressor) {
        compressor.enableDigital();
      } else {
        compressor.disable();
      }
    }
  }

  @Override
  public void runTests() {
    Test.add(this, "Compressor - Connected", !pcm.getCompressorNotConnectedStickyFault());
    Test.add(this, "Compressor - Current Low", !pcm.getCompressorCurrentTooHighStickyFault());  // Max continuous 12V / 17A
    Test.add(this, "Compressor - Not Shorted", !pcm.getCompressorShortedStickyFault());
    Test.add(this, "Battery - Voltage", RobotController.getBatteryVoltage() > 12.5);
  }
}
