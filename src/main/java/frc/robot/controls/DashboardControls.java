package frc.robot.controls;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.feeder.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.sequence.*;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.SmartSubsystem;

public class DashboardControls {
  public DashboardControls() {
    // TODO if not at event/competition

    addTuningCommands();

    addLayout("Commands", RobotContainer.SHOOTER.getName(), RobotContainer.SHOOTER, new ShooterTune(), new ShooterOpenLoop(.5));
    addLayout("Commands", RobotContainer.INTAKE.getName(), RobotContainer.INTAKE, new IntakeExtend(true));
    addLayout("Commands", RobotContainer.FEEDER.getName(), RobotContainer.FEEDER, new FeederOpenLoop(.5));
    addLayout("Commands", "Sequence", new ShootOpenLoop(), new DeployIntake(), new LoadCargo(), new StowIntake());
  }

  public void addTuningCommands() {
    SmartDashboard.putNumber("tuning/shooterVel", 0);
    SmartDashboard.putData("tuning/setSetpoints", new ShooterTune());
  }

  /**
   * Add a List layout to a tab
   * @param tab
   * @param name
   * @return
   */
  public static ShuffleboardLayout addLayout(String tab, String name) {
    ShuffleboardLayout layout = Shuffleboard.getTab(tab)
      .getLayout(name, BuiltInLayouts.kList)
      .withSize(2, 6)
      .withProperties(Map.of("Label position", "HIDDEN"));

    return layout;
  }

  /**
   * Add a List of cmd/subsystem/sendable items to a tab
   * @param tab
   * @param name
   * @param items
   * @return
   */
  public static ShuffleboardLayout addLayout(String tab, String name, Sendable... items) {
    ShuffleboardLayout layout = addLayout(tab, name);

    for (Sendable item : items) {
      layout.add(item);
    }

    return layout;
  }

}
