package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretOpenLoop extends InstantCommand {
  private final Turret subsystem = RobotContainer.TURRET;

  private final double percent;

  public TurretOpenLoop(double percent) {
    addRequirements(subsystem);
    this.percent = percent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(percent);
  }
}
