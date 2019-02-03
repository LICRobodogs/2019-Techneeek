package frc.util;

public class Constants {
    //DriveTrain
    public static final double DRIVER_JOY1_C1 = .0089;
    public static final double DRIVER_JOY1_C2 = .0737;
    public static final double DRIVER_JOY1_C3 = 2.4126;

    public static double m_moveInput = 0.0;
    public static double m_steerInput = 0.0;

	public static final double ENCODER_TICKS_TO_INCHES = 4096 * Math.PI * 4.0;
	public static final int DRIVE_TICKS_PER_FOOT = 3978; //Move robot 10 feet, get position from sensor, divide by 10
    //PID for left motor
	public static final double LEFT_P = 1.0;
	public static final double LEFT_I = 0.0;
	public static final double LEFT_D = 0.0;
	public static final double LEFT_F = 1023/5816;
    //PID for right motor
	public static final double RIGHT_P = 1.0;
	public static final double RIGHT_I = 0.0;
	public static final double RIGHT_D = 0.0;
	public static final double RIGHT_F = 1023/5574;
    //Controller
    public static final int leftTriggerAxis = 3;
    public static final int rightTriggerAxis = 4;
    public static final int leftAnalogAxis = 0;
    public static final double DEADZONE = 0.10;
    public static final int squareIndex = 1;
    public static final int xIndex = 2;
    public static final int circleIndex = 3;
    public static final int triangleIndex = 4;
    public static final int lbIndex = 5;
	public static final int rbIndex = 6;

    //Intake

    //Looper
    public static final double kLooperDt = 0.005;
    public static final int kMSTimeoutsFast = 10;
    public static final int kTimeoutMs = 20;
    public static final int kTalonRetryCount = 3;

    //FUck if I know, used in Kinematics
    public static double kTrackWidthInches = 2.0;
    public static double kTrackScrubFactor = 2.0;
    public static final double kSensorUnitsPerRotation = 4096.0;
    public static final double k100msPerMinute = 600.0;
    
    public static final double kSegmentCompletionTolerance = 1; // inches
    public static final double kPathFollowingMaxAccel = 100.0; // inches per second^2
    
    //WHEELS
    public static final double kDriveWheelDiameterInches = 4.875;	//Practice bot calibrated 4.875
	//public static final double kDriveWheelDiameterInches = 5;	//Comp bot measured val
    // public static final double kDriveWheelDiameterInches = PathAdapter.getAdaptedWheelDiameter();
    
	/* CONTROL LOOP GAINS */

	// PID gains for drive velocity loop (HIGH GEAR)
	// Units: setpoint, error, and output are in inches per second.
	public static final double kDriveHighGearVelocityKp = 1;
	public static final double kDriveHighGearVelocityKi = 0.005;
	public static final double kDriveHighGearVelocityKd = 1.6;
	public static final double kDriveHighGearVelocityKf = 0.165;
	public static final int kDriveHighGearVelocityIZone = 0;
	public static final double kDriveHighGearVelocityRampRate = 0.1;
    public static final double kDriveHighGearMaxSetpoint = 12.0 * 12.0; // 12 fps
    
    	// Path following constants
	public static final double kMinLookAhead = 12.0; // inches
	public static final double kMinLookAheadSpeed = 9.0; // inches per second
	public static final double kMaxLookAhead = 24.0; // inches
	public static final double kMaxLookAheadSpeed = 140.0; // inches per second
	public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;
    public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
	// our speed
	// in inches per sec
	// public static final double kSegmentCompletionTolerance = 1; // inches
	// public static final double kPathFollowingMaxAccel = 100.0; // inches per second^2
	public static final double kPathFollowingMaxVel = 140.0; // inches per second

	public static final double kPathFollowingProfileKp = 5.0;   //Used to be 5 when tuning our paths
	public static final double kPathFollowingProfileKi = 0.03;
	public static final double kPathFollowingProfileKv = 0.2;
	public static final double kPathFollowingProfileKffv = 1.0;
	public static final double kPathFollowingProfileKffa = 0.05;
	public static final double kPathFollowingGoalPosTolerance = 1;
	public static final double kPathFollowingGoalVelTolerance = 18.0;
	public static final double kPathStopSteeringDistance = 9.0;

		//Thread prioritization - 5 is default
		
		public static final int kLooperThreadPriority = Thread.MAX_PRIORITY;
		


}
