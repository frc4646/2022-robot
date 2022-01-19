package frc.robot.controls;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.shooter.ShooterTune;

public class DashboardControls {
  public DashboardControls() {
    // TODO if not at event/competition
    addTuningCommands();
  }

  public void addTuningCommands() {
    SmartDashboard.putNumber("tuning/shooterVel", 0);
    SmartDashboard.putData("tuning/setSetpoints", new ShooterTune());
  }
}
