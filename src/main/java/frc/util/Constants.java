package frc.util;
/**
 * The Constants is a mapping of all Constant values, ...from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Constants {
    // USB Port IDs
	public static final int DRIVER_GAMEPAD_USB_ID = 0;
	public static final int OPERATOR_GAMEPAD_USB_ID = 1;

	// LIMIT SWITCHES
	public static final int ELEVATOR_HOME_LIMIT_PORT = 0;

	// MOTORS
	public static final int DRIVETRAIN_LEFT_MOTOR1_ID = 2;
	public static final int DRIVETRAIN_LEFT_MOTOR2_ID = 3;
	public static final int DRIVETRAIN_RIGHT_MOTOR1_ID = 4;
	public static final int DRIVETRAIN_RIGHT_MOTOR2_ID = 5;

	public static final int TOP_INTAKE_TALON_ID = 2;
	public static final int BOTTOM_INTAKE_VICTOR_ID = 7;

	public static final int ELEVATOR_TALON1_ID = 5;
	public static final int ELEVATOR_TALON2_ID = 6;
    public static final int ELEVATOR_VICTOR1_ID = 8;
    
	public static final int WRIST_TALON_ID = 1;
	public static final int WRIST_VICTOR_ID = 9;

	public static final int LEFT_SUCTION_TALON_ID = 3;
	public static final int RIGHT_SUCTION_TALON_ID = 4;


	// PNEUMATICS
	public static final int BRAKE_DEPLOY_PCM_ID = 5;
	public static final int BRAKE_RELEASE_PCM_ID = 4;

	public static final int LEFT_SUCTION_PCM_ID = 0;
	public static final int RIGHT_SUCTION_PCM_ID = 1;

	public static final int SHOOT_IN_PCM_ID = 3;
	public static final int SHOOT_OUT_PCM_ID = 2;

	public static final int SHIFT_ELEVATOR_PCM_ID = 6;
    public static final int SHIFT_FORK_PCM_ID = 7;
	
	//Loops
	public static final int kTimeoutMs = 20;    
	public static final int kTalonRetryCount = 3;
	
    //Intake -- need units
    public static final double INTAKE_LOAD_SPEED = 0.65;
    public static final double INTAKE_EJECT_SPEED = -0.55;
	
	//Kinematics
	public static final double kSensorUnitsPerRotation = 4096.0;
    public static final double k100msPerMinute = 600.0;

    //Arm
    public final double SCALE_ANGLE_SETPOINT = 230;
	public final double SWITCH_ANGLE_SETPOINT = 80;
	public static double mArmOnTargetTolerance = 10;
	public static double mArmUpKp = 4.5;// .45
	public static double mArmUpKi = 0.0;
	public static double mArmUpKd = 0.0;// .25
	public static double mArmUpKf = 0.0;
	public static int mArmUpIZone = (int) (1023.0 / mArmUpKp);

	public static double mArmDownKp = 4;// .45
	public static double mArmDownKi = 0.0;
	public static double mArmDownKd = 0.0;// .25
	public static double mArmDownKf = 0.0;
	public static int mArmDownIZone = (int) (1023.0 / mArmDownKp);

	public static double mArmRampRate = .2;
    public static final double ARM_MOTOR_VOLTAGE_PERCENT_LIMIT = 4.0 / 12.0;
    public static final double ARM_NATIVE_TO_ANGLE_FACTOR = 10.0;
	public static final int START_CONFIG_ANGLE = 0; //FOR TESTING WHILE ROBOT INTAKE IS ON GROUND
	// public static final int START_CONFIG_ANGLE = 1900; //UNCOMMENT IF YOU WANT TO SIMULATE FULL MATCH
	public static final double ARM_HOLDING_PWM = 0.3;

	//Elevator
	public static final double ELEVATOR_NATIVE_TO_ANGLE_FACTOR = (100 / 12) * (84 / 14) * (84 / 32) * (20 / 32) * (16 / 18);
    public static final int HATCH_LEVEL1_SETPOINT = 4000;
    public static final int HATCH_LEVEL2_SETPOINT = 26500;
    public static final int HATCH_LEVEL3_SETPOINT = 49000;

	public static final int CARGO_LEVEL1_SETPOINT = 9500;
	public static final int CARGO_LEVEL2_SETPOINT = 31000;
    public static final int CARGO_LEVEL3_SETPOINT = 51500;

    //DT PID
    // public static final double ENCODER_TICKS_TO_INCHES = 4096 * Math.PI * 6.0;
    public static final double ENCODER_TICKS_TO_INCHES = 4096 * Math.PI * 4.0; //radius of wheel needed
	public static final int DRIVE_TICKS_PER_FOOT = 3978; //Move robot 10 feet, get position from sensor, divide by 10

	public static final double LEFT_P = 1.0;
	public static final double LEFT_I = 0.0;
	public static final double LEFT_D = 0.0;
	public static final double LEFT_F = 1023/5816;

	public static final double RIGHT_P = 1.0;
	public static final double RIGHT_I = 0.0;
	public static final double RIGHT_D = 0.0;
    public static final double RIGHT_F = 1023/5574;
    

    // Input Device Constants

	//ps4 controller
	///********************* */
	 //Controller
	//  public static final int leftTriggerAxis = 3;
	//  public static final int rightTriggerAxis = 4;
	//  public static final int leftAnalogAxis = 0;
	//  public static final double DEADZONE = 0.10;
	 public static final int PS_SQUARE_BUTTON = 3;
	 public static final int PS_X_BUTTON = 1;
	 public static final int PS_CIRCLE_BUTTON = 2;
	 public static final int PS_TRIANGLE_BUTTON = 4;
	 public static final int PS_LB_BUTTON = 5;
	 public static final int PS_RB_BUTTON = 6;


	public static final double DRIVER_JOY1_C1 = .0089;
	public static final double DRIVER_JOY1_C2 = .0737;
	public static final double DRIVER_JOY1_C3 = 2.4126;

	public static double m_moveInput = 0.0;
    public static double m_steerInput = 0.0;
    

    public static final int LEFT_X_AXIS = 0;
	public static final int LEFT_Y_AXIS = 1;
	public static final int LEFT_TRIGGER_AXIS = 2;
	public static final int LEFT_PS4_TRIGGER_AXIS = 3;
	public static final int RIGHT_PS4_TRIGGER_AXIS = 4;
	public static final int RIGHT_TRIGGER_AXIS = 3;
	public static final int RIGHT_X_AXIS = 4;
	public static final int RIGHT_Y_AXIS = 5;
	// public static final int LEFT_RIGHT_DPAD_AXIS = 6;
	// public static final int TOP_BOTTOM_DPAD_AXIS = 7; // Can't use this
	// axis... the Driver Station only transmits 6 axes.

	public static final int A_BUTTON = 1;
	public static final int B_BUTTON = 2;
	public static final int X_BUTTON = 3;
	public static final int Y_BUTTON = 4;
	public static final int LEFT_BUMPER_BUTTON = 5;
	public static final int RIGHT_BUMPER_BUTTON = 6;
	public static final int BACK_BUTTON = 7;
	public static final int START_BUTTON = 8;

	public static final int LEFT_JOYSTICK_BUTTON = 9;
	public static final int RIGHT_JOYSTICK_BUTTON = 10;

	// private static final double LEFT_DPAD_TOLERANCE = -0.9;
	// private static final double RIGHT_DPAD_TOLERANCE = 0.9;
	// private static final double BOTTOM_DPAD_TOLERANCE = -0.9;
	// private static final double TOP_DPAD_TOLERANCE = 0.9;

	public static final double LEFT_TRIGGER_TOLERANCE = 0.5;
	public static final double RIGHT_TRIGGER_TOLERANCE = 0.5;

	public static final double RIGHT_AXIS_UP_TOLERANCE = -0.9;
	public static final double RIGHT_AXIS_DOWN_TOLERANCE = 0.9;
	public static final double RIGHT_AXIS_RIGHT_TOLERANCE = 0.9;
	public static final double RIGHT_AXIS_LEFT_TOLERANCE = -0.9;

	public static final double LEFT_AXIS_UP_TOLERANCE = -0.9;
	public static final double LEFT_AXIS_DOWN_TOLERANCE = 0.9;
	public static final double LEFT_AXIS_RIGHT_TOLERANCE = 0.9;
	public static final double LEFT_AXIS_LEFT_TOLERANCE = -0.9;

    public static final double DEADZONE = 0.025;
    

    //Gamepad Trigger

	public static final int RIGHT_TRIGGER = 3;
	public static final int LEFT_TRIGGER = 2;
	// public static final int RIGHT_AXIS_UP_TRIGGER = 2;
	// public static final int RIGHT_AXIS_DOWN_TRIGGER = 3;
	public static final int RIGHT_AXIS_RIGHT_TRIGGER = 4;
	public static final int RIGHT_AXIS_LEFT_TRIGGER = 5;
	public static final int LEFT_AXIS_UP_TRIGGER = 6;
	public static final int LEFT_AXIS_DOWN_TRIGGER = 7;
	public static final int LEFT_AXIS_RIGHT_TRIGGER = 8;
	public static final int LEFT_AXIS_LEFT_TRIGGER = 9;
	

	//GameField
	public static final double TARGET_PORT_HEIGHT = 39.125; //inches | front of rocket ship
	public static final double TARGET_HATCH_HEIGHT = 31.0; //inches  | rocket side && loading station && cargo ship
	public static final double TARGET_PORT_RANGE = 1.0; // inches away from target for scoring
	public static final double TARGET_HATCH_RANGE = 23.0; // inches away from target for scoring
	
	//LimeLight
	public static final double CAMERA_HEIGHT = 32.00;
	public static final double CAMERA_MOUNT_ANGLE = 0.0;
	public static final double kp_ll = 1.0;
	public static final double HORIZONTAL_FOV = 54; // degrees
	public static final double VERTICAL_FOV = 41; // degrees
	
	//Drivetrain
	public static final double KpSteer = 0.0375; // 0.04 Proportional control constant for steering
	public static final double KpDrive = 0.02; // 0.0175 Proportional control constant for steering


    
}