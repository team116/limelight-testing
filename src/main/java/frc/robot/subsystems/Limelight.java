package frc.robot.subsystems;

import java.util.*;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
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

    public static final double LIMELIGHT_HEIGHT = 31 + 5.0d/16; // Height of lens from floor (in inches) TODO: Test this value
    public static final double LIMELIGHT_ANGLE = 0; // Downwards pitch (in degrees) TODO: Test this value

    private HashMap<Integer, Double> heightMap = new HashMap<>();

    private static final double TRENCH_HEIGHT = 35.0d;
    private static final double HUB_HEIGHT = 44.25d;
    private static final double OUTPOST_HEIGHT = 21.75d;
    private static final double TOWER_HEIGHT = 21.75d;

    private static final int[] TRENCH_TAGS = new int[] {1, 12, 6, 7, 17, 28, 22, 23};
    private static final int[] HUB_TAGS = new int[] {3, 4, 5, 8, 9, 10, 11, 2, 18, 19, 20, 21, 24, 25, 26, 27};
    private static final int[] OUTPOST_TAGS = new int[] {13, 14, 29, 30};
    private static final int[] TOWER_TAGS = new int[] {15, 16, 31, 32};


    NetworkTableInstance limelighTableInstance;
    NetworkTable limelightTable;
    int streamModeToggleValue = STREAM_MODE_STANDARD;
    int currentStreamMode;
    final DoubleSubscriber taSubscriber;
    final DoubleSubscriber txSubscriber;
    final DoubleSubscriber tySubscriber;
    final IntegerSubscriber tidSubscriber;

    public Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_HOST_NAME);
        taSubscriber = limelightTable.getDoubleTopic("ta").subscribe(1.0);
        txSubscriber = limelightTable.getDoubleTopic("tx").subscribe(1.0);
        tySubscriber = limelightTable.getDoubleTopic("ty").subscribe(1.0);
        tidSubscriber = limelightTable.getIntegerTopic("tid").subscribe(1);

        configureAprilTagBindings();
    }

    private void configureAprilTagBindings() {
        massInsert(TRENCH_HEIGHT, TRENCH_TAGS);
        
        massInsert(HUB_HEIGHT, HUB_TAGS);

        massInsert(OUTPOST_HEIGHT, OUTPOST_TAGS);
        
        massInsert(TOWER_HEIGHT, TOWER_TAGS);
    }

    private void massInsert(double value, int[] keys) {
        for(int i : keys) {
            heightMap.put(i, value);
        }
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

    public int getTid() {
        return (int)(tidSubscriber.get());
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

    public double getDistanceFromAprilTagWithID() {
        int id = getTid();
        double angle = getTy();
        if(heightMap.containsKey(id)) {
            return (heightMap.get(id) - LIMELIGHT_HEIGHT)/(Math.tan(Math.toRadians(LIMELIGHT_ANGLE + angle))); // TODO: Test this formula
        } else {
            return getDistanceFromAprilTagInches();
        }
    }

    public double getDistanceFromAprilTagFeet() {
        return getDistanceFromAprilTagInches() / 12;
    }
}