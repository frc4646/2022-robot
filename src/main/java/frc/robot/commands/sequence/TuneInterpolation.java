package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.commands.shooter.ShooterRevDistance;
import frc.robot.commands.shooter.ShooterWaitForVelocity;

public class TuneInterpolation extends SequentialCommandGroup {
  public TuneInterpolation() {
    addCommands(
      new ShooterOpenLoop(0.525),  // TODO try ShooterTune with Smartdashboard
      //new ShooterWaitForVelocity(2200.0),
      //new ShooterRevDistance(127.0),
      new WaitCommand(3.0),  // TODO try parallel print of voltage & rpm
      new FeederOpenLoop(Constants.Feeder.PERCENT_OPEN_LOOP),
      new WaitCommand(1.0)
    );
  }
}
