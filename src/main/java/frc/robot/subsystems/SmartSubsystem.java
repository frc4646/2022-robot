package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SmartSubsystem extends SubsystemBase {
  /** 
   * Override to store sensor readings.
   * <i>Synchronizes sensor values</i> across subsystems each cycle of the scheduler by being called <i>before</i> all subsystems run commands.
   */
  public void cacheSensors() {};

  /** 
   * Override to optionally set outputs after commands.
   * <i>Improves consistency</i> of command execution by being called <i>after</i> all subsystems run commands.
   */
  public void updateHardware() {};
  
  /** 
   * Override to update SmartDashboard values.
   * <i>Improves consistency</i> of command execution by being called <i>after</i> all subsystems run commands.
   */
  public void updateDashboard(boolean showDetails) {};

  /**
   * Override to do something whenever the <i>robot becomes enabled</i>.
   */
  public void onEnable(boolean isAutonomous) {
  }

  /**
   * Override to do something whenever the <i>robot becomes disabled</i>.
   */
  public void onDisable() {}

   /**
   * Override to check subsystem when enabling in test mode.
   */
  public void runTests() {}

  // @Override
  // public void initSendable(SendableBuilder builder) { /* Do Nothing */ }
}