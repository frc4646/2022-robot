package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.Constants;

public class Pneumatics extends SmartSubsystem {

  PneumaticsControlModule pcm;

  private NetworkTableEntry guiPressureSwitch;
  
  public Pneumatics() {
    pcm = new PneumaticsControlModule(Constants.CAN.PNEUMATIC_CONTROL_MODULE);

    ShuffleboardLayout layout = Shuffleboard.getTab("General").getLayout("Pneumatics", BuiltInLayouts.kList).withSize(2, 4);
    guiPressureSwitch = layout.add("Pressure Switch", pcm.getPressureSwitch()).getEntry();
  }

  @Override
  public void updateDashboard()
  {
    guiPressureSwitch.setBoolean(pcm.getPressureSwitch());
  }

}
