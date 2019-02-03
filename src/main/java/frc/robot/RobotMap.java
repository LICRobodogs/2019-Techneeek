package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
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

	public static final int TOP_INTAKE_TALON_ID = 1;
	public static final int BOTTOM_INTAKE_VICTOR_ID = 7;

	public static final int ELEVATOR_TALON1_ID = 1;
	public static final int ELEVATOR_TALON2_ID = 2;
    public static final int ELEVATOR_VICTOR1_ID = 1;
    
	public static final int WRIST_TALON_ID = 3;
	public static final int WRIST_VICTOR_ID = 2;

	public static final int LEFT_SUCTION_TALON_ID = 4;
	public static final int RIGHT_SUCTION_TALON_ID = 5;


	// PNEUMATICS
	public static final int BRAKE_DEPLOY_PCM_ID = 2;
	public static final int BRAKE_RELEASE_PCM_ID = 3;

	public static final int LEFT_SUCTION_PCM_ID = 0;
	public static final int RIGHT_SUCTION_PCM_ID = 1;

	public static final int SHOOT_IN_PCM_ID = 4;
	public static final int SHOOT_OUT_PCM_ID = 5;

	public static final int SHIFT_ELEVATOR_PCM_ID = 6;
	public static final int SHIFT_FORK_PCM_ID = 7;

}