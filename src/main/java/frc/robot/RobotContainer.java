package frc.robot;

import frc.robot.commands.drivetrain.DriveTeleop;
import frc.robot.controls.Controls;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  public static Climber CLIMBER;
  public static Drivetrain DRIVETRAIN;
  public static Indexer INDEXER;
  public static Intake INTAKE;
  public static Shooter SHOOTER;
  public static Vision VISION;

  public static Controls CONTROLS;

  private final Command autoCommand = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //CLIMBER = new Climber();
    DRIVETRAIN = new Drivetrain();
    //INDEXER = new Indexer();
    INTAKE = new Intake();
    SHOOTER = new Shooter();
    //VISION = new Vision();

    CONTROLS = new Controls();  // Create after subsystems
    DRIVETRAIN.setDefaultCommand(new DriveTeleop());
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
