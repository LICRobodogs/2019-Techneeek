package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Singleton Class LimeLight that instantiates the camera & provides
 * functionality
 */
public class LimeLight extends Subsystem{
    private NetworkTable table;
    public NetworkTableEntry tX_offset, tY_offset, tArea, tSkew, tValid, tLatency, tShort_length, tLong_length, tHor_length, tVert_length, getPipe, camtran;
    private NetworkTableEntry ledMode, camMode, setPipe, streamMode, snapshot;
    private static LimeLight instance = null;
    private static final double accepted_error_angle = 0.5;
    private static final double accepted_error_distance = 0.5;

    public static enum LED {
        DEFAULT, OFF, BLINK, ON
    }

    public static enum CameraMode {
        VISION_PROCESSOR, DRIVER_CAMERA
    }

    public static enum StreamMode {
        STANDARD, PIP_MAIN, PIP_SECONDARY
    }
    public static enum TargetType {
        PORT, HATCH
    }

    private LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        initRetrievableData();
        initSetableData();

    }
    private void initRetrievableData() {
        tValid = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
        tX_offset = table.getEntry("tx"); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
        tY_offset = table.getEntry("ty"); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
        tArea = table.getEntry("ta"); // Target Area (0% of image to 100% of image)
        tSkew = table.getEntry("ts"); // Target Skew, not sure of value range
        tLatency = table.getEntry("tl"); //The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
        tShort_length = table.getEntry("tl"); //Sidelength of shortest side of the fitted bounding box (pixels)
        tLong_length = table.getEntry("tlong"); //Sidelength of longest side of the fitted bounding box (pixels)
        tHor_length = table.getEntry("thor"); //Horizontal sidelength of the rough bounding box (0 - 320 pixels
        tVert_length = table.getEntry("tvert"); //Vertical sidelength of the rough bounding box (0 - 320 pixels)
        getPipe = table.getEntry("getpipe"); // True active pipeline index of the camera (0 .. 9)
        camtran = table.getEntry("camtran"); // Results of a 3D position solution, 6 numbers: Translation (x,y,y) Rotation(pitch,yaw,roll)
    }
    private void initSetableData() {
        ledMode = table.getEntry("ledMode"); //sets limelights LED state
        camMode = table.getEntry("camMode"); //sets limelights operation mode
        setPipe = table.getEntry("pipeline"); //sets limelights current pipeline
        streamMode = table.getEntry("stream"); // sets limelight's streaming mode
        snapshot = table.getEntry("snapshot"); //allows snapshots
    }

    public static LimeLight getInstance() {
        if (instance == null)
            instance = new LimeLight();
        return instance;
    }

    /**
     * post tv, ts, tx, & ty data to Smart Dashboard from network table
     */
    public void getBasicData() {
        double v = tValid.getDouble(0.0);
        double s = tSkew.getDouble(0.0);
        double x = tX_offset.getDouble(0.0);
        double y = tY_offset.getDouble(0.0);
        double area = tArea.getDouble(0.0);
        setData(v, s, x, y, area);
    }
    public void getAllData() {
    }
    
	@Override
	protected void initDefaultCommand() {
	}

    /**
     * post to smart dashboard periodically
     */
    public void setData(double v, double s, double x, double y, double area) {
        SmartDashboard.putNumber("Valid Target", v);
        SmartDashboard.putNumber("LimelightSkew", s);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    public void setLEID(LED state) {
        switch (state) {
        case DEFAULT:
        ledMode.setNumber(0);
            break;
        case OFF:
        ledMode.setNumber(1);
            break;
        case BLINK:
        ledMode.setNumber(2);
            break;
        case ON:
        ledMode.setNumber(3);
            break;
        default:
            System.err.println("UNKNOWN LEID STATE PASSED.");
            break;
        }
    }
    /** 
     * starts taking 2 snapshots per second
     */
    public void startTakingSnapshots() {
        snapshot.setNumber(1); //two per second
    }
    /**
     * stops taking snapshots
     */
    public void stopTakingSnapshots() {
        snapshot.setNumber(0); //two per second
    }

    public void switchCameraMode(CameraMode mode) {
        switch (mode) {
        case VISION_PROCESSOR:
            camMode.setNumber(0);
            break;
        case DRIVER_CAMERA:
            camMode.setNumber(1);
            break;
        default:
            System.err.println("ERROR SETTING CAMERA MODE");
            break;
        }
    }

    public void setStream(StreamMode mode) {
        switch (mode) {
        case STANDARD:
            streamMode.setNumber(0);
            break;
        case PIP_MAIN:
            streamMode.setNumber(1);
            break;
        case PIP_SECONDARY:
            streamMode.setNumber(2);
            break;
        default:
            System.err.println("ERROR SETTING STREAM MODE");
            break;
        }
    }
    public double getPipeline() {
        return getPipe.getDouble(0);
    }
    
    public void setPipeline(double pipe) {
        setPipe.setNumber(pipe);
    }

    /**
     * Determines the distance to a target using geometry
     * 
     * @param h1 height of camera
     * @param h2 height of target
     * @param a1 y angle of camera mount
     * @param a2 y angle of target from crosshair of camera
     * @return distance to a target
     */
    public double distanceToTarget(double h1, double h2, double a1, double a2) {
        double distance = (h2 - h1) / Math.tan(a1 + a2);
        return distance;
    }

    /**
     * {@link #distanceToTarget(double, double, double, double)}
     * 
     * @return distance to Hatch Target
     */
    public double distanceToHatch() {
        return distanceToTarget(Constants.CAMERA_HEIGHT, Constants.TARGET_HATCH_HEIGHT, Constants.CAMERA_MOUNT_ANGLE,
                tY_offset.getDouble(0.0));
    }

    /**
     * {@link #distanceToTarget(double, double, double, double)}
     * 
     * @return distance to Port Target
     */
    public double distanceToPort() {
        return distanceToTarget(Constants.CAMERA_HEIGHT, Constants.TARGET_PORT_HEIGHT, Constants.CAMERA_MOUNT_ANGLE,
                tY_offset.getDouble(0.0));
    }

    /**
     * Spins robot until a target is in view, then centers on it
     */
    public void seekTarget() {
        if (tValid.getDouble(0.0) == 0.0) {
            Robot.driveTrain.drive(0.3);
        } else {
            aimAtTarget();
        }
    }

    /**
     * Aim at the largest target recognized in the limelight pipeline
     */
    public void aimAtTarget() {
        double KpAim = 0.1; // Proportional control constant
        double min_power = 0.05;
        double min_threshold = 1.0;
        double steering_adjust = 0.0;
        double xAngle = tX_offset.getDouble(0.0);
        if (Math.abs(xAngle) > min_threshold) {
            steering_adjust = KpAim * xAngle - min_power;
        } else {
            steering_adjust = KpAim * xAngle + min_power;
        }
        Robot.driveTrain.drive(steering_adjust);
    }

    /**
     * {@link #getInRange(double, double)}
     * get within a constant distance of a hatch for scoring
     */
    public void getInHatchRange() {
        getInRange(Constants.TARGET_HATCH_RANGE, distanceToHatch());
    }

    /**
     * {@link #getInRange(double, double)}
     * get within a constant distance of port for scoring
     */
    public void getInPortRange() {
        getInRange(Constants.TARGET_PORT_RANGE, distanceToPort());
    }

    /**
     * drives to robots desired location based on distance to target
     * 
     * @param desired_distance inches of distance desired 
     * @param current_distance inches of current distance
     */
    public void getInRange(double desired_distance, double current_distance) {
        double KpDistance = 0.1; // Proportional control constant for distance
        double distance_error = current_distance - desired_distance;
        double driving_adjust = KpDistance * distance_error;
        Robot.driveTrain.setSpeed(driving_adjust, driving_adjust);
    }

    /**
     * To use, put robot at desired distance & calibrate the y-position of the
     * crosshair. This allows the angle to report "0.0" when at desired distance
     */
    public void getInRange() {
        double KpDistance = 0.1; // Proportional control constant for distance
        double driving_adjust = KpDistance * tY_offset.getDouble(0.0);
        Robot.driveTrain.setSpeed(driving_adjust, driving_adjust);
    }

    /**
     * aim at hatch target & drive towards it
     */
    public void aimAndDrive() {
        double KpAim = 0.1; // Proportional control constant
        double KpDistance = 0.1; // Proportional control constant
        double min_power = 0.05;
        double min_threshold = 1.0; // used for angle & distance..for now

        double steering_adjust = 0.0;
        double heading_error = tX_offset.getDouble(0.0);
        double distance_error = distanceToHatch() - min_threshold; // = ty.getDouble(0.0); //only if calibrated to do so

        if (Math.abs(heading_error) > min_threshold) {
            steering_adjust = KpAim * heading_error - min_power;
        } else {
            steering_adjust = KpAim * heading_error + min_power;
        }
        double distance_adjust = KpDistance * distance_error;

        Robot.driveTrain.drive(distance_adjust, steering_adjust);
    }

    public double xAngle_toPixelLocation(PixelCoord pixel) {
        return angle_toPixelLocation(pixel.getNormalized().getViewPlaneCoordinates().getX());
    }

    public double yAngle_toPixelLocation(PixelCoord pixel) {
        return angle_toPixelLocation(pixel.getNormalized().getViewPlaneCoordinates().getY());
    }
    public double angle_toPixelLocation(double loc) {
        return Math.atan2(1, loc);
    }
    public boolean isAtTarget(TargetType target) {
        switch(target) {
            case HATCH:
            return distanceToHatch() <= accepted_error_distance;
            case PORT:
            return distanceToPort() <= accepted_error_distance;
            default:
            System.err.println("INVALID TARGET TYPE PASSED");
            return true;
        }
    }
    public boolean isAimed() {
        return tX_offset.getDouble(0.0) <= accepted_error_angle;
    }

}

/**
 * Helper class in case we need to start translating coordinates or mapping
 * coordinates onto virtual plane --Purpose of 3 classes is remapping raw pixels
 * into coordinate locations then determining their location on a virtual plan
 * and uses the two computed coordinate locations to determine angles
 */
class Coordinate {
    private double x, y;

    public Coordinate() {
        this(0.0, 0.0);
    }

    public Coordinate(double x, double y) {
        set(x, y);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        set(x, this.y);
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        set(this.x, y);
    }

    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }
}

/**
 * pixel coordinates, 0,0 is the upper-left, positive down and to the righ
 */
class PixelCoord extends Coordinate {
    public PixelCoord(double x, double y) {
        super(x, y);
    }

    /**
     * Normalize the X coordinate with respect to 320 horizontal fov
     */
    public double getX_normalized() {
        return (1 / 160) * (getX() - ((Constants.HORIZONTAL_FOV - 1) / 2));
    }

    /**
     * Normalize the Y coordinate with respect to 240 vertical fov
     */
    public double getY_normalized() {
        return (1 / 120) * (getY() - ((Constants.VERTICAL_FOV - 1) / 2));
    }

    public NormalCoordinate getNormalized() {
        return new NormalCoordinate(getX_normalized(), getY_normalized());
    }
}

/**
 * normalized pixel coordinates, 0,0 is the center, positive right and up
 */
class NormalCoordinate extends Coordinate {
    public NormalCoordinate(double x, double y) {
        super(x, y);
    }

    public double getX_ViewPlaneCoord() {
        double x = getViewPlaneWidth() / 2 * getX();
        return x;
    }

    public double getY_ViewPlaneCoord() {
        double y = getViewPlaneHeight() / 2 * getY();
        return y;
    }

    public Coordinate getViewPlaneCoordinates() {
        return new Coordinate(getX_ViewPlaneCoord(), getY_ViewPlaneCoord());
    }

    private double getViewPlaneWidth() {
        return 2.0 * Math.tan(Constants.HORIZONTAL_FOV / 2);
    }

    private double getViewPlaneHeight() {
        return 2.0 * Math.tan(Constants.VERTICAL_FOV / 2);
    }
}