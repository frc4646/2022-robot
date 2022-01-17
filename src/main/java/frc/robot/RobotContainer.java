package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveTeleop;
import frc.robot.controls.Controls;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  public static DriveSubsystem DRIVE;
  
  public static Controls CONTROLS;

  private final Command autoCommand = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DRIVE = new DriveSubsystem();
    CONTROLS = new Controls();  // Create after subsystems

    DRIVE.setDefaultCommand(new DriveTeleop());
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // TODO
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
