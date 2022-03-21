package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class FeederAutoIndex extends SequentialCommandGroup {
  private static final Feeder feeder = RobotContainer.FEEDER;
  
  public FeederAutoIndex() {
    addRequirements(feeder);
    addCommands(
      new SelectCommand(FeederAutoIndex::select)
    );
  }

  public static Command select() {
    if (!feeder.isShooterLoaded() && !feeder.isCargoIndexed() && feeder.isHooperFull()) {
      return new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_LOAD).until(() -> { return feeder.isCargoIndexed() || feeder.isShooterLoaded(); });
    }
    return new FeederOpenLoop().perpetually().until(() -> { return feeder.isShooterLoaded() || feeder.isCargoIndexed() || feeder.isHooperFull(); });
  }
}
