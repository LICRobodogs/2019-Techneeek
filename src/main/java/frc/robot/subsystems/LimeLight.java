package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Constants;

/**
 * Singleton Class LimeLight that instantiates the camera & provides
 * functionality
 */
public class LimeLight extends Subsystem {
    private NetworkTable table;
    public NetworkTableEntry tX_offset, tY_offset, tArea, tSkew, tValid, tLatency, tShort_length, tLong_length,
            tHor_length, tVert_length, getPipe, camtran;
    private NetworkTableEntry ledMode, camMode, setPipe, streamMode, snapshot;
    private static LimeLight instance = null;
    Double[] previosValues = new Double[6];
    double desiredYaw = 0;

    public static enum LED {
        DEFAULT, OFF, BLINK, ON
    }

    public static enum CameraMode {
        VISION_PROCESSOR, DRIVER_CAMERA
    }

    public static enum StreamMode {
        STANDARD, PIP_MAIN, PIP_SECONDARY
    }

    private LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        initRetrievableData();
        initSetableData();
        desiredYaw = 0.0;
    }

    private void initRetrievableData() {
        tValid = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
        tX_offset = table.getEntry("tx"); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
        tY_offset = table.getEntry("ty"); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
        tArea = table.getEntry("ta"); // Target Area (0% of image to 100% of image)
        tSkew = table.getEntry("ts"); // Target Skew, -90 -> 0 degrees
        tLatency = table.getEntry("tl"); // The pipeline’s latency contribution (ms) Add at least 11ms for image capture
                                         // latency.
        tShort_length = table.getEntry("tl"); // Sidelength of shortest side of the fitted bounding box (pixels)
        tLong_length = table.getEntry("tlong"); // Sidelength of longest side of the fitted bounding box (pixels)
        tHor_length = table.getEntry("thor"); // Horizontal sidelength of the rough bounding box (0 - 320 pixels
        tVert_length = table.getEntry("tvert"); // Vertical sidelength of the rough bounding box (0 - 320 pixels)
        getPipe = table.getEntry("getpipe"); // True active pipeline index of the camera (0 .. 9)
        camtran = table.getEntry("camtran"); // Results of a 3D position solution, 6 numbers: Translation (x,y,y)
                                             // Rotation(pitch,yaw,roll)
    }

    private void initSetableData() {
        ledMode = table.getEntry("ledMode"); // sets limelights LED state
        camMode = table.getEntry("camMode"); // sets limelights operation mode
        setPipe = table.getEntry("pipeline"); // sets limelights current pipeline
        streamMode = table.getEntry("stream"); // sets limelight's streaming mode
        snapshot = table.getEntry("snapshot"); // allows snapshots
    }

    public static LimeLight getInstance() {
        if (instance == null)
            instance = new LimeLight();
        return instance;
    }

    @Override
    protected void initDefaultCommand() {
    }

    /**
     * post to smart dashboard periodically
     */
    public void postAllData() {
        SmartDashboard.putNumber("Valid Target", tValid.getDouble(0.0));
        // SmartDashboard.putNumber("LimelightSkew", tSkew.getDouble(0.0));
        // SmartDashboard.putNumber("LimelightX", tX_offset.getDouble(0.0));
        // SmartDashboard.putNumber("LimelightY", tY_offset.getDouble(0.0));
        // SmartDashboard.putNumber("LimelightArea", tArea.getDouble(0.0));
        Double[] emptyDouble = {};
        // SmartDashboard.putNumberArray("CAMTRAN", camtran.getDoubleArray(emptyDouble));
        SmartDashboard.putNumber("real Distance", getCamtranDistance());
        SmartDashboard.putNumber("real YAW", getCamtranYaw());
    }

    public double getCamtranDistance() {
        Double[] values = getCamtranValues();
        if (values.length != 1)
            return -values[2];
        return 0.0;
    }

    public double getCamtranYaw() {
        Double[] values = getCamtranValues();
        if (values.length != 1)
            return values[4];
        return 0.0;
    }
    public double getCamtranX() {
        Double[] values = getCamtranValues();
        if (values.length != 1)
            return values[0];
        return 0.0;
    }

    public Double[] getCamtranValues() {
        Double[] emptyDouble = { -1000.0 };
        return camtran.getDoubleArray(emptyDouble);
    }

    public double getPipeline() {
        return getPipe.getDouble(0);
    }

    public void setPipeline(double pipe) {
        setPipe.setNumber(pipe);
    }

    public boolean isSeeingTarget() {
        return tValid.getDouble(0.0) != 0.0;
    }

    public boolean is3dCompute() {
        return getCamtranValues().length > 1;
    }

    // public void drive_and_steer() {
    //     if (isNeedingTurn()) {
    //         if(getCamtranX()>0) {
    //             Robot.driveTrain.setSpeed(-.2, .2);
    //         } else {
    //             Robot.driveTrain.setSpeed(.2, -.2);
    //         }
    //     } else {
    //         Robot.driveTrain.arcadeDrive(0.7 * -getDrivingAdjustment(), 0.7 * -getSteeringAdjustment());
    //     }
    // }
    public void drive_and_steer() {
        
            // Robot.driveTrain.arcadeDrive(0.7 * -getDrivingAdjustment(), 0.7 * -(getSteeringAdjustment()*getCamtranX()));
            // Robot.driveTrain.arcadeDrive(0.7 * -getDrivingAdjustment(), Constants.KpSteer * -getCamtranX());
            Robot.driveTrain.arcadeDrive(0.7 * -getDrivingAdjustment(), Constants.KpSteer * -getSteeringAdjustment());
    }
    public void setDesiredYaw(double desired) {
        this.desiredYaw = desired;
    }
    public double getDesiredYaw() {
        return (this.desiredYaw == 0.0) ? -getCamtranYaw() : this.desiredYaw;
    }

    // TODO try with feed forward
    public double getDrivingAdjustment() {
        if (isAtTarget()) {
            return 0.0;
        }
        double distance_error = getCamtranDistance() - Constants.DESIRED_STOPPING_DISTANCE;
        double driving_adjust = distance_error * Constants.KpDrive;
        return driving_adjust;
    }
/*
    public double getSteeringAdjustment() {
        if (isAimedAtTarget()) {
            return 0.0;
        }
        double steer_adjust = getCamtranYaw() * Constants.KpSteer;
        return steer_adjust;
    }
    */
    public boolean isNeedingTurn() {
        double x = getCamtranX();
        double yaw = getCamtranYaw();
        double distance = getCamtranDistance();
        SmartDashboard.putNumber("X: Camtran",x);

        
        // return (((yaw > 0) != (x > 0)) || ((Math.abs(x) > 4) && distance > 40)) ;
        return ((yaw > 0) != (x > 0));
    }

    public double getSteeringAdjustment() {
        double x = getCamtranX();
        double yaw = getCamtranYaw();
        if ((yaw > 0) == (x > 0)) {
            return 0;
        }
        double steer_adjust = getCamtranYaw() * Constants.KpSteer;
        return steer_adjust;
    }

    public void driveStraightToTarget() {
        Robot.driveTrain.arcadeDrive(0.7 * -getDrivingAdjustment(), 0);
    }

    public void aimAtTarget() {
        Robot.driveTrain.arcadeDrive(0, -getSteeringAdjustment());
    }

    /**
     * Return true when robot is within an error distance of target
     */
    public boolean isAtTarget() {
        double distance_error = Math.abs(getCamtranDistance() - Constants.DESIRED_STOPPING_DISTANCE);
        return distance_error <= Constants.ACCEPTED_DISTANCE_ERROR;
    }

    public boolean isAimedAtTarget() {
        double alignment_error = Math.abs(getCamtranYaw()) - Constants.DESIRED_STOPPING_ANGLE;
        return Math.abs(alignment_error) <= Constants.ACCEPTED_ANGLE_ERROR;
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
        snapshot.setNumber(1); // two per second
    }

    /**
     * stops taking snapshots
     */
    public void stopTakingSnapshots() {
        snapshot.setNumber(0); // two per second
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

}
