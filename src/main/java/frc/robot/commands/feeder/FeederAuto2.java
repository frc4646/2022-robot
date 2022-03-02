package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.sequence.ShootExhaust;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Feeder;

public class FeederAuto2 {
  /** Default command of Feeder subsystem */
  public static class FeederAuto extends SelectCommand {
    public FeederAuto() {
      super(FeederAuto2::nextCommand);
    }
  }

  private static final double TURNS_ENQUEUE = 0.75;
  private static final double TURNS_EXHAUST = 0.75;
  private static final ColorSensor colorSensor = RobotContainer.COLOR_SENSOR;
  private static final Feeder feeder = RobotContainer.FEEDER;

  private static Command nextCommand() {
    // Feeder subsystem default command must only require feeder, therefore ScheduleCommand needed to fork requirements of shoot commands.
    if (feeder.getQueuedCargoCorrect() >= 2) {
      return new ScheduleCommand(new ShootVision()).andThen(new WaitForColorSense());  // Can remove if we always want on button
    } else if (feeder.getQueuedCargoWrong() >= 1) {
      return new ScheduleCommand(new ShootExhaust()).andThen(new WaitForColorSense());  // TODO have ShootVision do wrong cargo too?
    } else if (colorSensor.isCorrectCargo() || colorSensor.isWrongCargo() && feeder.getQueuedCargo() == 0) {
      return new FeederPosition(TURNS_ENQUEUE);
    } else if (colorSensor.isWrongCargo() && feeder.getQueuedCargoCorrect() > 0) {
      return new FeederPosition(-TURNS_EXHAUST);
    }
    return new WaitForColorSense();
  }

  private static boolean isActionWanted() {
    return colorSensor.isCargoPresent() || feeder.getQueuedCargoCorrect() >= 2 || feeder.getQueuedCargoWrong() >= 1;
  }

  private static class WaitForColorSense extends ParallelDeadlineGroup {
    public WaitForColorSense() {
      super(new WaitUntilCommand(FeederAuto2::isActionWanted), new FeederPosition(0.0));
    }
  }
}
