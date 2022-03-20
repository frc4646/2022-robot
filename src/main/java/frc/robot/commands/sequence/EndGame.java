package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.VisionLED;
import frc.robot.commands.climber.ClimberExtend;
import frc.robot.commands.climber.ClimberPosition;
import frc.robot.subsystems.Vision.LEDMode;

public class EndGame extends SequentialCommandGroup {
  private static final double HEIGHT_CLAWS_SPRING_UP = 0.5;
  private static final double HEIGHT_CLAWS_SUPPORT_ROBOT = HEIGHT_CLAWS_SPRING_UP + 0.1;
  private static final double HEIGHT_NEXT_RUNG_REACH_OUT = 1.0;
  private static final double HEIGHT_NEXT_RUNG_SUPPORT_ROBOT = 0.9;

  public EndGame() {
    addCommands(
      // Assume drive team already has climber on mid rung
      new ClimberPosition(HEIGHT_CLAWS_SPRING_UP),
      pauseBetweenEachStep(),
      new ClimberPosition(HEIGHT_CLAWS_SUPPORT_ROBOT),
      pauseBetweenEachStep(),
      new ClimberExtend(true),
      // TODO wait for pitch?
      pauseBetweenEachStep(),
      new ClimberPosition(HEIGHT_NEXT_RUNG_REACH_OUT),
      pauseBetweenEachStep(),
      new ClimberPosition(HEIGHT_NEXT_RUNG_SUPPORT_ROBOT),
      pauseBetweenEachStep()
      // TODO 
    );
  }

  public Command pauseBetweenEachStep() {
    return sequence(
      new VisionLED(LEDMode.BLINK),
      new WaitCommand(15.0),
      new VisionLED(LEDMode.ON),
      new WaitCommand(5.0),
      new VisionLED(LEDMode.OFF)
    );
  }
}
