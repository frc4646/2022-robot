package frc.robot.controls;

public class Controls {
    public final DriverControls driver;
    public final OperatorControls operator;

    public Controls() {
        driver = new DriverControls();
        operator = new OperatorControls();
    }
}
