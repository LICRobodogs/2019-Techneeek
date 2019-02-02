package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.util.*;
import frc.robot.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem implements ControlLoopable {
	public static enum DriveTrainControlMode {
		JOYSTICK, AUTON, HOLD, TEST
	};

	// Input Device Constants

	public static final double DRIVER_JOY1_C1 = .0089;
	public static final double DRIVER_JOY1_C2 = .0737;
	public static final double DRIVER_JOY1_C3 = 2.4126;

	private double m_moveInput = 0.0;
	private double m_steerInput = 0.0;

	// NavX

	// private AHRS mxp = new AHRS(SPI.Port.kMXP);

	// Set control mode
	private DriveTrainControlMode controlMode = DriveTrainControlMode.JOYSTICK;

	// Robot Intrinsics
	public static double m_periodMs;

	public static final double ENCODER_TICKS_TO_INCHES = 4096 * Math.PI * 4.0;
	public static final int DRIVE_TICKS_PER_FOOT = 3978; //Move robot 10 feet, get position from sensor, divide by 10

	public static final double LEFT_P = 1.0;
	public static final double LEFT_I = 0.0;
	public static final double LEFT_D = 0.0;
	public static final double LEFT_F = 1023/5816;

	public static final double RIGHT_P = 1.0;
	public static final double RIGHT_I = 0.0;
	public static final double RIGHT_D = 0.0;
	public static final double RIGHT_F = 1023/5574;

    private CANSparkMax leftDrive1;
    private CANSparkMax leftDrive2;
    private CANSparkMax rightDrive1;
    private CANSparkMax rightDrive2;

	private DifferentialDrive m_drive;

	@SuppressWarnings("unused")
	private static boolean isRunning;

	public DriveTrain() {
		try {
			leftDrive1 = new CANSparkMax(2,CANSparkMaxLowLevel.MotorType.kBrushless);
            leftDrive2 = new CANSparkMax(3,CANSparkMaxLowLevel.MotorType.kBrushless);
            rightDrive1 = new CANSparkMax(4,CANSparkMaxLowLevel.MotorType.kBrushless);
            rightDrive2 = new CANSparkMax(5,CANSparkMaxLowLevel.MotorType.kBrushless);
            leftDrive2.follow(leftDrive1);
            rightDrive2.follow(rightDrive1);
			
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