package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.util.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Singleton Class LimeLight that instantiates the camera & provides
 * functionality
 */
public class LimeLight {
    private NetworkTable table;
    private NetworkTableEntry tx, ty, ta, ts, tv;
    private static LimeLight instance = null;

    private LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx"); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
        ty = table.getEntry("ty"); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
        ta = table.getEntry("ta"); // Target Area (0% of image to 100% of image)
        ts = table.getEntry("ts"); // Target Skew, not sure of value range
        tv = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
    }

    public static LimeLight getInstance() {
        if (instance == null)
            instance = new LimeLight();
        return instance;
    }
/**
 * post tv, ts, tx, & ty data to Smart Dashboard from network table
 */
    public void getData() {
        double v = tv.getDouble(0.0);
        double s = ts.getDouble(0.0);
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        setData(v, s, x, y, area);

    }
/**
 * // post to smart dashboard periodically
 */
    public void setData(double v, double s, double x, double y, double area) {
        SmartDashboard.putNumber("Valid Target", v);
        SmartDashboard.putNumber("LimelightSkew", s);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
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
                ty.getDouble(0.0));
    }

    /**
     * {@link #distanceToTarget(double, double, double, double)}
     * 
     * @return distance to Port Target
     */
    public double distanceToPort() {
        return distanceToTarget(Constants.CAMERA_HEIGHT, Constants.TARGET_PORT_HEIGHT, Constants.CAMERA_MOUNT_ANGLE,
                ty.getDouble(0.0));
    }

    /**
     * Spins robot until a target is in view, then centers on it
     */
    public void seekTarget() {
        if (tv.getDouble(0.0) == 0.0) {
            // We don't see the target, seek for the target by spinning in place at a safe
            // speed.
            DriveTrain.getInstance().drive(0.3);
        } else {
            // We do see the target, execute aiming code
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
        double xAngle = tx.getDouble(0.0);
        if (Math.abs(xAngle) > min_threshold) {
            steering_adjust = KpAim * xAngle - min_power;
        } else {
            steering_adjust = KpAim * xAngle + min_power;
        }
        DriveTrain.getInstance().drive(steering_adjust);
    }

    /**
     * get within a constant distance of a hatch for scoring
     */
    public void getInHatchRange() {
        getInRange(Constants.TARGET_HATCH_RANGE, distanceToHatch());
    }

    /**
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
        DriveTrain.getInstance().setSpeed(driving_adjust, driving_adjust);
    }

    /**
     * To use, put robot at desired distance & calibrate the y-position of the
     * crosshair. This allows the angle to report "0.0" when at desired distance
     */
    public void getInRange() {
        double KpDistance = 0.1; // Proportional control constant for distance
        double driving_adjust = KpDistance * ty.getDouble(0.0);
        DriveTrain.getInstance().setSpeed(driving_adjust, driving_adjust);
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
        double heading_error = tx.getDouble(0.0);
        double distance_error = distanceToHatch() - min_threshold; // = ty.getDouble(0.0); //only if calibrated to do so

        if (Math.abs(heading_error) > min_threshold) {
            steering_adjust = KpAim * heading_error - min_power;
        } else {
            steering_adjust = KpAim * heading_error + min_power;
        }
        double distance_adjust = KpDistance * distance_error;

        DriveTrain.getInstance().drive(distance_adjust, steering_adjust);
    }

    public double xAngle_toPixelLocation(PixelCoord pixel) {
        Coordinate coord = pixel.getNormalized().getViewPlaneCoordinates();
        return Math.atan2(1, coord.getX());
    }

    public double yAngle_toPixelLocation(PixelCoord pixel) {
        Coordinate coord = pixel.getNormalized().getViewPlaneCoordinates();
        return Math.atan2(1, coord.getY());
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