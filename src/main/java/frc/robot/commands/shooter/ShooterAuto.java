package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterAuto extends CommandBase {
  private final Shooter subsystem = RobotContainer.SHOOTER;

  public ShooterAuto() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double setpoint = 0.0;

    // TODO if loaded cargo wrong, use exhaust rpm?
    // TODO if both cargo correct, prepare using minimum viable rpm?

    subsystem.setClosedLoop(setpoint);
  }
}
