package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.wait.WaitForColorState;
import frc.robot.commands.wait.WaitForShooterLoaded;
import frc.robot.subsystems.ColorSensor.STATE;

public class FeederIndexCargo extends ParallelDeadlineGroup {
  private static final double TURNS_ENQUEUE = 0.75;

  public FeederIndexCargo() {
    super(
      race(
        new WaitUntilCommand(FeederIndexCargo::hasTwoCargo),
        new WaitForShooterLoaded(true)  // Don't index when already loaded
      ),
      sequence(
        new WaitForColorState(STATE.CORRECT),
        new FeederPosition(TURNS_ENQUEUE),
        new WaitForColorState(STATE.CORRECT),
        new FeederPosition(TURNS_ENQUEUE),
        new PrintCommand("ERROR: FeederIndex didn't detect two cargo then interrupt"),  // TODO remove after testing
        new PrintCommand("ERROR: FeederIndex didn't detect two cargo then interrupt 2")  // TODO remove after testing
      )
    );
  }

  private static boolean hasTwoCargo() {
    return false;  // TODO
  }
}
