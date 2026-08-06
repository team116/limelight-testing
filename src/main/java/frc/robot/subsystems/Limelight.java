package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private static final String LIMELIGHT_HOST_NAME = "limelight";

    public static final int STREAM_MODE_STANDARD = 0;  // Primary and secondary cameras shown side-by-side
    public static final int STREAM_MODE_PIP_MAIN = 1;  // Picture in picture with secondary camera in lower right corner
    public static final int STREAM_MODE_PIP_SECONDARY = 2; // Picture in picture with primary camera in lower right corner

    public static final int LED_BY_CURRENT_PIPELINE = 0;
    public static final int LED_FORCE_OFF = 1;
    public static final int LED_FORCE_BLINK = 2;
    public static final int LED_FORCE_ON = 3;

    NetworkTableInstance limelighTableInstance;
    NetworkTable limelightTable;
    int streamModeToggleValue = STREAM_MODE_STANDARD;
    int currentStreamMode;
    final DoubleSubscriber taSubscriber;
    final DoubleSubscriber txSubscriber;
    final DoubleSubscriber tySubscriber;

    public Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_HOST_NAME);
        taSubscriber = limelightTable.getDoubleTopic("ta").subscribe(1.0);
        txSubscriber = limelightTable.getDoubleTopic("tx").subscribe(1.0);
        tySubscriber = limelightTable.getDoubleTopic("ty").subscribe(1.0);
    }

    public void toggleStreamMode() {
        setStreamMode(currentStreamMode == STREAM_MODE_PIP_MAIN ? STREAM_MODE_PIP_SECONDARY : STREAM_MODE_PIP_MAIN);
    }

    public void setStreamMode(int newStreamMode) {
        limelightTable.getEntry("stream").setNumber(newStreamMode);
        currentStreamMode = newStreamMode;
    }

    public void setLedMode(int newLedMode) {
        limelightTable.getEntry("ledMode").setNumber(newLedMode);
    }

    public void ledOn() {
        setLedMode(LED_FORCE_ON);
    }

    public void ledOff() {
        setLedMode(LED_FORCE_OFF);
    }

    public boolean hasValidTarget() {
        // SmartDashboard.putNumber("ta", taSubscriber.get());
        return taSubscriber.get() > 0.05d;
    }

    public double targetAreaPercentageOfImage(){
        // SmartDashboard.putNumber("ta", taSubscriber.get());
        return taSubscriber.get();
    }

    public double getTa() {
        return taSubscriber.get();
    }

    public double getTx() {
        return txSubscriber.get();
    }

    public double getTy() {
        return tySubscriber.get();
    }

    public double horizontalOffsetFromCrosshairAsDegrees() {
        // SmartDashboard.putNumber("tx", txSubscriber.get());
        return txSubscriber.get();
    }

    public double getDistanceFromAprilTagInches() {
        return getDistanceFromAprilTagInches(getTa());
    }
    
    public static double getDistanceFromAprilTagInches(double ta) {
        return (68.86483 * Math.pow(ta, -0.633957)); // return (185.267402 * Math.pow(ta, -0.4997426126));
    }

    public double getDistanceFromAprilTagFeet() {
        return getDistanceFromAprilTagInches() / 12;
    }
}