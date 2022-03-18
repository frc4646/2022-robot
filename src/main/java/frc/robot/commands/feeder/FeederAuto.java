package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class FeederAuto extends CommandBase {
  private final Feeder feeder = RobotContainer.FEEDER;

  public FeederAuto() {
    addRequirements(feeder);
  }

  @Override
  public void initialize() {
    feeder.setOpenLoop(0.0);
  }
}
