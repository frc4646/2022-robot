package frc.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.RobotState;

public class AgitatorAuto extends SequentialCommandGroup {
  private static final RobotState state = RobotContainer.ROBOT_STATE;
  private static final Agitator agitator = RobotContainer.AGITATOR;
  
  public AgitatorAuto() {
    addRequirements(agitator);
    addCommands(new SelectCommand(AgitatorAuto::select));
  }

  public static Command select() {
    if (state.isAgiateHopperWanted()) {
      return new AgitatorPulse().until(RobotContainer.ROBOT_STATE::isAgiateHopperFinished);
    } else if (state.isIndexingWanted()) {
      return new AgitatorOpenLoop(Constants.AGITATOR.OPEN_LOOP_LOAD).until(RobotContainer.ROBOT_STATE::isIndexingFinished);
    }
    return new AgitatorOpenLoop().perpetually().until(RobotContainer.ROBOT_STATE::isAnyCargoPresent);
  }
}
