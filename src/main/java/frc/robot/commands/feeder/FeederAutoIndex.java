package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.RobotState;

public class FeederAutoIndex extends SequentialCommandGroup {
  private static final RobotState state = RobotContainer.ROBOT_STATE;
  private static final Feeder feeder = RobotContainer.FEEDER;
  
  public FeederAutoIndex() {
    addRequirements(feeder);
    addCommands(new SelectCommand(FeederAutoIndex::select));
  }

  public static Command select() {
    if (state.isIndexingWanted()) {
      return new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_LOAD).until(RobotContainer.ROBOT_STATE::isIndexingFinished);
    }
    return new FeederOpenLoop().perpetually().until(RobotContainer.ROBOT_STATE::isAnyCargoPresent);
    // if (state.isAgiateHopperWanted()) {
    //   return parallel(new AgitatorPulse(), new FeederOpenLoop()).until(RobotContainer.ROBOT_STATE::isAgiateHopperFinished);
    // } else if (state.isIndexingWanted()) {
    //   return parallel(new AgitatorOpenLoop(Constants.AGITATOR.OPEN_LOOP_LOAD), new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_LOAD)).until(RobotContainer.ROBOT_STATE::isIndexingFinished);
    // }
    // return new IndexCargo(false, false).perpetually().until(RobotContainer.ROBOT_STATE::isAnyCargoPresent);
  }
}
