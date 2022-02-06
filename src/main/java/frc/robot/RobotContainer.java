package frc.robot;

import frc.robot.commands.drivetrain.DriveTeleop;
import frc.robot.controls.AutoModeSelector;
import frc.robot.controls.Controls;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SmartSubsystem;
import frc.robot.subsystems.Turret;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  public static Agitator AGITATOR;
  public static Climber CLIMBER;
  public static Drivetrain DRIVETRAIN;
  public static Feeder FEEDER;
  public static Hood HOOD;
  public static Intake INTAKE;
  public static Shooter SHOOTER;
  public static Turret TURRET;
  public static Vision VISION;
  public static Pneumatics PNEUMATICS;
  private final List<SmartSubsystem> allSubsystems;

  public static Controls CONTROLS;

  public final AutoModeSelector autoModeSelector;

  private final NetworkTableEntry guiVoltage;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AGITATOR = new Agitator();
    // CLIMBER = new Climber();
    DRIVETRAIN = new Drivetrain();
    FEEDER = new Feeder();
    // HOOD = new Hood();
    INTAKE = new Intake();
    SHOOTER = new Shooter();
    // TURRET = new Turret();
    VISION = new Vision();
    PNEUMATICS = new Pneumatics();
    allSubsystems = Arrays.asList(AGITATOR, DRIVETRAIN, FEEDER, INTAKE, SHOOTER, VISION, PNEUMATICS);

    CONTROLS = new Controls();  // Create after subsystems
    DRIVETRAIN.setDefaultCommand(new DriveTeleop());

    autoModeSelector = new AutoModeSelector();

    guiVoltage = Shuffleboard.getTab("General").add("Voltage", RobotController.getBatteryVoltage()).withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("min", 6, "max", 14)).getEntry();
  }

  public void cacheSensors() {
    allSubsystems.forEach(SmartSubsystem::cacheSensors);
  }

  public void updateDashboard() {
    allSubsystems.forEach(SmartSubsystem::updateDashboard);

    guiVoltage.setDouble(RobotController.getBatteryVoltage());
  }

  public void runTests() {
    allSubsystems.forEach(SmartSubsystem::runTests);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
