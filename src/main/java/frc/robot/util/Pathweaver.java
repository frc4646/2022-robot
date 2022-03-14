package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class Pathweaver {
  public static Trajectory getTrajectory(String name)  {
    Trajectory trajectory = null;
    try {
      if(!name.endsWith(".wpilib.json")) {
        name = name + ".wpilib.json";
      }
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + name);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + name, ex.getStackTrace());
    }
    return trajectory;
  }
}
