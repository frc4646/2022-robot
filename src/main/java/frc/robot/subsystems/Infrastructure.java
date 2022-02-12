package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.Constants;
import frc.robot.util.Test;

public class Infrastructure extends SmartSubsystem {
  private final Compressor compressor;
  private final PneumaticsControlModule pcm;
  private NetworkTableEntry dashPressureSwitch, dashCompressor, dashVoltage;

  public Infrastructure() {
    compressor = new Compressor(Constants.CAN.PNEUMATIC_CONTROL_MODULE, PneumaticsModuleType.CTREPCM);
    pcm = new PneumaticsControlModule(Constants.CAN.PNEUMATIC_CONTROL_MODULE);
    
    ShuffleboardLayout layout = Shuffleboard.getTab("General").getLayout("Pneumatics", BuiltInLayouts.kList).withSize(2, 4);
    dashPressureSwitch = layout.add("Pressure Switch", pcm.getPressureSwitch()).getEntry();
    dashCompressor = layout.add("Compressor", compressor.enabled()).getEntry();    
    dashVoltage = Shuffleboard.getTab("General").add("Voltage", RobotController.getBatteryVoltage()).withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("min", 6, "max", 14)).getEntry();
  }
  
  /** Enable/disable the {@link Compressor} closed loop, which <i>automatically</i> runs the {@link Compressor} when pressure is low */
  public void setCompressor(boolean enable) {
      if(enable) {
          compressor.enableDigital();
      }
      else {
          compressor.disable();
      }
  }
  
  @Override
  public void updateDashboard() {
    dashPressureSwitch.setBoolean(pcm.getPressureSwitch());
    dashCompressor.setBoolean(compressor.enabled());
    dashVoltage.setDouble(RobotController.getBatteryVoltage());
  }

  @Override
  public void runTests() {
    boolean isCurrentLowEnough = !pcm.getCompressorCurrentTooHighStickyFault();  // Max continuous 12V / 17A
    boolean isConnected = !pcm.getCompressorNotConnectedStickyFault();
    boolean isNotShorted = !pcm.getCompressorShortedStickyFault();
    boolean isBatteryFull = RobotController.getBatteryVoltage() > 13.0;
    pcm.clearAllStickyFaults();
    
    System.out.println(String.format("Compressor connected: %s", Test.getResultString(isConnected)));
    System.out.println(String.format("Compressor current: %s", Test.getResultString(isCurrentLowEnough)));
    System.out.println(String.format("Compressor shorted: %s", Test.getResultString(isNotShorted)));
    System.out.println(String.format("Battery voltage: %s", Test.getResultString(isBatteryFull)));
  }
}
