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
  private final DataCache cache = new DataCache();
  // private final UsbCamera camera;

  // private NetworkTableEntry dashPressureSwitch, dashCompressor, dashVoltage;

  public Infrastructure() {
    compressor = new Compressor(Constants.CAN.PNEUMATIC_CONTROL_MODULE, PneumaticsModuleType.CTREPCM);
    pcm = new PneumaticsControlModule(Constants.CAN.PNEUMATIC_CONTROL_MODULE);
    // camera = CameraServer.startAutomaticCapture();

    // ShuffleboardLayout layout = Shuffleboard.getTab("General").getLayout("Pneumatics", BuiltInLayouts.kList).withSize(2, 4);
    // dashPressureSwitch = DashboardControls.getGraph(tab, "Pressure Switch", 0.0).getEntry();
    // dashCompressor = DashboardControls.getGraph(tab, "Compressor", 0.0).getEntry();
    // dashVoltage = DashboardControls.getGraph(tab, "Voltage", 0.0).withProperties(Map.of("min", 6, "max", 14)).getEntry();
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
    // dashPressureSwitch.setBoolean(pcm.getPressureSwitch());
    // dashCompressor.setBoolean(compressor.enabled());
    // dashVoltage.setDouble(RobotController.getBatteryVoltage());    
    SmartDashboard.putNumber("Battery", cache.battery);
  }

  @Override
  public void runTests() {
    boolean isCurrentLowEnough = !pcm.getCompressorCurrentTooHighStickyFault();  // Max continuous 12V / 17A
    boolean isConnected = !pcm.getCompressorNotConnectedStickyFault();
    boolean isNotShorted = !pcm.getCompressorShortedStickyFault();
    boolean isBatteryFull = RobotController.getBatteryVoltage() > 13.0;
    // boolean isCameraEnabled = camera.isEnabled();

    Test.add(this, "Compressor - Connected", isConnected);
    Test.add(this, "Compressor - Current", isCurrentLowEnough);
    Test.add(this, "Compressor - Shorted", isNotShorted);
    Test.add(this, "Battery - Voltage", isBatteryFull);
    // Test.add(this, "Camera - Enabled", isCameraEnabled);
  }
}
