package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.JoystickDrive;
import frc.util.Constants;
import frc.util.ControlLoopable;

public class DriveTrain extends Subsystem implements ControlLoopable {
	private static DriveTrain instance = null;
	
	public static DriveTrain getInstance() {
		if (instance == null)
			instance = new DriveTrain();
		return instance;
	}

	
	public static enum DriveTrainControlMode {
		JOYSTICK, AUTON, HOLD, TEST
	};

	// Input Device Constants

	public static final double DRIVER_JOY1_C1 = Constants.DRIVER_JOY1_C1;
	public static final double DRIVER_JOY1_C2 = Constants.DRIVER_JOY1_C2;
	public static final double DRIVER_JOY1_C3 = Constants.DRIVER_JOY1_C3;

	private double m_moveInput = Constants.m_moveInput;
	private double m_steerInput = Constants.m_steerInput;
	private double desiredSpeed = 0.4;

	// NavX

	// private AHRS mxp = new AHRS(SPI.Port.kMXP);

	// Set control mode
	private DriveTrainControlMode controlMode = DriveTrainControlMode.JOYSTICK;

	// Robot Intrinsics
	public static double m_periodMs;

	public static final double ENCODER_TICKS_TO_INCHES = Constants.ENCODER_TICKS_TO_INCHES;
	public static final int DRIVE_TICKS_PER_FOOT = Constants.DRIVE_TICKS_PER_FOOT; //Move robot 10 feet, get position from sensor, divide by 10

	public static final double LEFT_P = Constants.LEFT_P;
	public static final double LEFT_I = Constants.LEFT_I;
	public static final double LEFT_D = Constants.LEFT_D;
	public static final double LEFT_F = Constants.LEFT_F;

	public static final double RIGHT_P = Constants.RIGHT_P;
	public static final double RIGHT_I = Constants.RIGHT_I;
	public static final double RIGHT_D = Constants.RIGHT_D;
	public static final double RIGHT_F = Constants.RIGHT_F;

    private CANSparkMax leftDrive1;
    private CANSparkMax leftDrive2;
    private CANSparkMax rightDrive1;
    private CANSparkMax rightDrive2;

	private DifferentialDrive m_drive;

	@SuppressWarnings("unused")
	private static boolean isRunning;

	private DriveTrain() {
		try {
			leftDrive1 = new CANSparkMax(2,CANSparkMaxLowLevel.MotorType.kBrushless);
            // leftDrive2 = new CANSparkMax(3,CANSparkMaxLowLevel.MotorType.kBrushless);
            rightDrive1 = new CANSparkMax(4,CANSparkMaxLowLevel.MotorType.kBrushless);
            // rightDrive2 = new CANSparkMax(5,CANSparkMaxLowLevel.MotorType.kBrushless);
			leftDrive1.setIdleMode(IdleMode.kBrake);
			// leftDrive2.setIdleMode(IdleMode.kBrake);
			rightDrive1.setIdleMode(IdleMode.kBrake);
			// rightDrive2.setIdleMode(IdleMode.kBrake);
			
			// leftDrive2.follow(leftDrive1);
            // rightDrive2.follow(rightDrive1);
			
			m_drive = new DifferentialDrive(leftDrive1, rightDrive1);

			m_drive.setSafetyEnabled(false);
		} catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}

	public void setSpeed(double speed) {
		setControlMode(DriveTrainControlMode.JOYSTICK, speed);
	}

	public void setSpeed(double speed1, double speed2) {
		// if (speed == 0) {
		/*
		 * } else { setControlMode(DriveTrainControlMode.TEST); rightDrive1.set(speed);
		 * leftDrive1.set(speed); }
		 */
		leftDrive1.set(speed1);
		rightDrive1.set(speed2);
	}
	
	public void driveWithJoystick() {
		if (controlMode != DriveTrainControlMode.JOYSTICK || m_drive == null)
			return;

		m_moveInput = 0.5*OI.getInstance().getMoveInput();
		m_steerInput = 0.5*OI.getInstance().getSteerInput();

		m_drive.curvatureDrive(m_moveInput, m_steerInput, true);
	}
	public void drive(double steer) {
		drive(0.5*OI.getInstance().getMoveInput(), steer);
	}
	public void drive(double move, double steer) {
		// m_drive.curvatureDrive(move, -steer, true); //dont remember why steer is negated?
		m_drive.curvatureDrive(move, steer, true);
	}

	@Override
    protected void initDefaultCommand() {
        setDefaultCommand(new JoystickDrive());
    }
	
	public void setControlMode(DriveTrainControlMode controlMode, double speed) {
		this.controlMode = controlMode;
		if (controlMode == DriveTrainControlMode.JOYSTICK) {
			leftDrive1.set(speed);
			rightDrive1.set(speed);
		} else if (controlMode == DriveTrainControlMode.AUTON) {
			leftDrive1.set(speed);
			rightDrive1.set(speed);
		}
	}

	@Override
	public void controlLoopUpdate() {
		if (controlMode == DriveTrainControlMode.JOYSTICK) {
			// driveWithJoystick();
		} else if (controlMode == DriveTrainControlMode.AUTON) {
			// executeMovement();
			//leftDrive1.getMotionProfileStatus(statusLeft);
			//rightDrive1.getMotionProfileStatus(statusRight);
		}

	}

	@Override
	public void setPeriodMs(long periodMs) {
		m_periodMs = periodMs;

	}

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.COMPETITION) {
			// SmartDashboard.putNumber("Current Left Robot Drive Position: ", leftDrive1.getSelectedSensorPosition(0));
			// SmartDashboard.putNumber("Current Right Robot Drive Position: ", rightDrive1.getSelectedSensorPosition(0));
			// SmartDashboard.putNumber("Current Left Robot Drive EncVelocity: ", leftDrive1.getSelectedSensorVelocity(0));
			// SmartDashboard.putNumber("Current Right Robot Drive EncVelocity: ",
			// 		rightDrive1.getSelectedSensorVelocity(0));
			// SmartDashboard.putNumber("Current Right Robot Drive Error: ", rightDrive1.getClosedLoopError(0));
			// SmartDashboard.putNumber("Current Left Robot Drive Error: ", leftDrive1.getClosedLoopError(0));
			// // SmartDashboard.putBoolean("isFinished", isFinished());
			// // SmartDashboard.putBoolean("isRunning", isRunning);
			// SmartDashboard.putString("Left Drive TALON MODE: ", leftDrive1.getControlMode().toString());
			// SmartDashboard.putString("Right Drive TALON MODE: ", rightDrive1.getControlMode().toString());
		}
	}

	public boolean isFinished() {
		return false;
	}

	public static void setRunning(boolean isRunning) {
		DriveTrain.isRunning = isRunning;
	}

	public double getDesiredSpeed(){
		return this.desiredSpeed;
	}

	public void setDesiredSpeed(double speed){
		this.desiredSpeed = speed;
	}
}