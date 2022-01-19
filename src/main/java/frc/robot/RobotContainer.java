package frc.robot;

import frc.robot.commands.drive.DriveTeleop;
import frc.robot.controls.Controls;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  public static ClimberSubsystem CLIMBER;
  public static DriveSubsystem DRIVE;
  public static IntakeSubsystem INTAKE;
  public static ShooterSubsystem SHOOTER;

  public static Controls CONTROLS;

  private final Command autoCommand = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //CLIMBER = new ClimberSubsystem();
    //DRIVE = new DriveSubsystem();
    INTAKE = new IntakeSubsystem();
    //SHOOTER = new ShooterSubsystem();

    CONTROLS = new Controls();  // Create after subsystems
    //DRIVE.setDefaultCommand(new DriveTeleop());
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
