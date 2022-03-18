package frc.robot;

import frc.robot.commands.CompressorAuto;
import frc.robot.commands.SignalDriveTeam;
import frc.robot.commands.agitator.AgitatorAuto;
import frc.robot.commands.climber.ClimberTeleop;
import frc.robot.commands.drivetrain.DriveTeleop;
import frc.robot.commands.drivetrain.DriveDisabled;
import frc.robot.commands.feeder.FeederAuto;
import frc.robot.commands.intake.IntakeAuto;
import frc.robot.commands.shooter.ShooterAuto;
import frc.robot.commands.turret.TurretAim;
import frc.robot.controls.AutoModeSelector;
import frc.robot.controls.Controls;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Canifier;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Infrastructure;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LEDMode;
import frc.team4646.Test;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.SmartSubsystem;
import frc.robot.subsystems.Turret;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  public static Agitator AGITATOR;
  public static Canifier CANIFIER;
  public static Climber CLIMBER;
  public static ColorSensor COLOR_SENSOR;
  public static Diagnostics DIAGNOSTICS;
  public static Drivetrain DRIVETRAIN;
  public static Feeder FEEDER;
  public static Intake INTAKE;
  public static Infrastructure INFRASTRUCTURE;
  public static Shooter SHOOTER;
  public static ShooterTop SHOOTER_TOP;
  public static Turret TURRET;
  public static Vision VISION;

  public static Controls CONTROLS;

  private final List<SmartSubsystem> allSubsystems;
  public final AutoModeSelector autoModeSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CONTROLS = new Controls();  // Must create controllers BEFORE subsystems
    AGITATOR = new Agitator();
    CANIFIER = new Canifier();
    CLIMBER = new Climber();
    COLOR_SENSOR = new ColorSensor();
    DRIVETRAIN = new Drivetrain();
    FEEDER = new Feeder();
    INFRASTRUCTURE = new Infrastructure();
    INTAKE = new Intake();
    SHOOTER = new Shooter();
    SHOOTER_TOP = new ShooterTop();
    TURRET = new Turret();
    VISION = new Vision();
    DIAGNOSTICS = new Diagnostics();
    allSubsystems = Arrays.asList(AGITATOR, CANIFIER, CLIMBER, COLOR_SENSOR, DRIVETRAIN, FEEDER, INFRASTRUCTURE, INTAKE, SHOOTER, SHOOTER_TOP, TURRET, VISION, DIAGNOSTICS);

    CONTROLS.configureButtons();  // Must create buttons AFTER subsystems
    DRIVETRAIN.setDefaultCommand(new DriveTeleop());
    INFRASTRUCTURE.setDefaultCommand(new CompressorAuto());
    DIAGNOSTICS.setDefaultCommand(new SignalDriveTeam());
    AGITATOR.setDefaultCommand(new AgitatorAuto());
    // CLIMBER.setDefaultCommand(new ClimberAuto());  // TODO try ClimberZero on button if this doesn't work
    CLIMBER.setDefaultCommand(new ClimberTeleop());
    FEEDER.setDefaultCommand(new FeederAuto());
    INTAKE.setDefaultCommand(new IntakeAuto());
    SHOOTER.setDefaultCommand(new ShooterAuto());
    TURRET.setDefaultCommand(new TurretAim());

    autoModeSelector = new AutoModeSelector();
  }

  public void cacheSensors() {
    allSubsystems.forEach(SmartSubsystem::cacheSensors);
  }

  public void updateDashboard() {
    boolean isCompetition =  DriverStation.isFMSAttached();
    boolean isForceButtonPressed = SmartDashboard.getBoolean(Constants.SHOW_DETAILS, false);
    boolean showDetails = !isCompetition || isForceButtonPressed;
    allSubsystems.forEach(s -> s.updateDashboard(showDetails));
  }

  public void onEnable(boolean isAutonomous) {
    for(SmartSubsystem subsystem : allSubsystems) {
      subsystem.onEnable(isAutonomous);
    }
  }

  public void onDisable() {
    allSubsystems.forEach(SmartSubsystem::onDisable);
    new DriveDisabled().schedule();
  }

  public void runTests() {
    if (VISION != null) {
      VISION.setLED(LEDMode.BLINK);
    }
    Test.reset();
    Timer.delay(3.0);
    if (VISION != null) {
      VISION.setLED(LEDMode.OFF);
    }
    allSubsystems.forEach(SmartSubsystem::runTests);
    Test.results();
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
