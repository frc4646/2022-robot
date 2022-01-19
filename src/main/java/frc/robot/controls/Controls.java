package frc.robot.controls;

public class Controls {
  public final DriverControls driver;
  public final OperatorControls operator;
  public final DashboardControls smartdashboard;

  public Controls() {
    driver = new DriverControls();
    operator = new OperatorControls();
    smartdashboard = new DashboardControls();
  }
}
