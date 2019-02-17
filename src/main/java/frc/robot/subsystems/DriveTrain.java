package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.JoystickDrive;
import frc.util.Constants;
import frc.util.ControlLoopable;
import frc.util.drivers.DunkTalonSRX;
import frc.util.drivers.DunkVictorSPX;

public class DriveTrain extends Subsystem implements ControlLoopable {
	public static enum DriveTrainControlMode {
		JOYSTICK, AUTON, HOLD, TEST
	};

	// Input Device Constants

	public static final double DRIVER_JOY1_C1 = Constants.DRIVER_JOY1_C1;
	public static final double DRIVER_JOY1_C2 = Constants.DRIVER_JOY1_C2;
	public static final double DRIVER_JOY1_C3 = Constants.DRIVER_JOY1_C3;

	private double m_moveInput = Constants.m_moveInput;
	private double m_steerInput = Constants.m_steerInput;

	// NavX

	// private AHRS mxp = new AHRS(SPI.Port.kMXP);

	// Set control mode
	private DriveTrainControlMode controlMode = DriveTrainControlMode.JOYSTICK;

	// Robot Intrinsics
	public static double m_periodMs;


    private DunkTalonSRX leftDrive1;
    private DunkTalonSRX leftDrive2;
    private DunkTalonSRX rightDrive1;
	private DunkVictorSPX rightDrive2;
	
	private AHRS mxp;

	private DifferentialDrive m_drive;

	// @SuppressWarnings("unused")
	private static boolean isRunning;

	public DriveTrain() {
		try {
			leftDrive1 = new DunkTalonSRX(0);
        	leftDrive2 = new DunkTalonSRX(3);
        	rightDrive1 = new DunkTalonSRX(1);
        	rightDrive2 = new DunkVictorSPX(2);
        	leftDrive2.follow(leftDrive1);
        	rightDrive2.follow(rightDrive1);
			m_drive = new DifferentialDrive(leftDrive1, rightDrive1);
			// mxp = new AHRS(SPI.Port.kMXP);

			m_drive.setSafetyEnabled(false);
		} catch (Exception e) {
			throw new RuntimeException("An error occurred in the Drivetrain constructor", e);
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

		m_moveInput = OI.getInstance().getMoveInput();
		m_steerInput = OI.getInstance().getSteerInput();

		m_drive.curvatureDrive(m_moveInput, m_steerInput, true);
	}
	
	public void drive(double move, double steer) {
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
			driveWithJoystick();
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
}