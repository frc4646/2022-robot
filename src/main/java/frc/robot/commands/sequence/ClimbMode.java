package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.climber.ClimberArms;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.turret.TurretPosition;
import frc.robot.commands.vision.VisionLED;
import frc.robot.subsystems.Vision.LEDMode;

public class ClimbMode extends ConditionalCommand {

  public ClimbMode() {
    super(new SetMode(LEDMode.OFF, true), new SetMode(LEDMode.ON, false), ClimbMode::nextMode);
  }

  public static boolean isClimbMode() {
    return inClimbMode;
  }
  
  private static class SetMode extends SequentialCommandGroup {
    public SetMode(LEDMode led, boolean extend) {
      addCommands(        
        new VisionLED(led),
        new TurretPosition(Constants.Turret.SERVO.kHomePosition, 0.1),
        new ClimberArms(extend),
        new IntakeExtend(extend)
        // TODO set drivetrain brake mode?
      );
    }
  }

  private static boolean inClimbMode = false;

  private static boolean nextMode() {
    inClimbMode = !inClimbMode;
    return inClimbMode;
  }
}
