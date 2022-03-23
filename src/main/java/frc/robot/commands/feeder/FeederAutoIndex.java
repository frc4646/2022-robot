package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.sequence.IndexCargo;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class FeederAutoIndex extends SequentialCommandGroup {
  private static final Feeder feeder = RobotContainer.FEEDER;
  private static final Intake intake = RobotContainer.INTAKE;
  
  public FeederAutoIndex() {
    addRequirements(feeder);
    addCommands(
      new SelectCommand(FeederAutoIndex::select)
    );
  }

  public static Command select() {
    if (intake.isExtended() && !feeder.isHopperFull()) {
      return new IndexCargo(false, true).until(() -> { return !intake.isExtended() || feeder.isHopperFull(); });
    } else if (!feeder.isShooterLoaded() && !feeder.isCargoIndexed() && feeder.isHopperFull()) {
      return new IndexCargo().until(() -> { return feeder.isCargoIndexed() || feeder.isShooterLoaded(); });
    }
    // else if (!feeder.isShooterLoaded() && feeder.isCargoIndexed() && feeder.isHooperFull()) {
    //   return new IndexCargo().until(() -> { return  feeder.isShooterLoaded(); });
    // }
    return new IndexCargo(false, false).perpetually().until(() -> { return feeder.isShooterLoaded() || feeder.isCargoIndexed() || feeder.isHopperFull(); });
  }
}
