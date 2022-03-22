package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;

public class ShooterAutoRev extends SequentialCommandGroup {
  private static RobotState state = RobotContainer.ROBOT_STATE;
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;

  public ShooterAutoRev() {
    addRequirements(shooter, shooterTop);
    addCommands(new SelectCommand(ShooterAutoRev::select));
  }

  public static Command select() {
    if (state.isAutoRevWanted()) {
      return new ShooterRev().until(() -> { return !state.isAutoRevWanted(); } );
    } 
    return new ShooterOpenLoop().perpetually().until(() -> { return state.isAutoRevWanted(); });
  }
}
