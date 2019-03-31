/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.intake.IntakeSuction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.LED;
import frc.util.Constants;
import frc.util.ControlLooper;

public class Robot extends TimedRobot {
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	public static DriveTrain driveTrain;
	public static Intake intake;
	public static Arm arm;
	public static Elevator elevator;
	public static Compressor comp;
	public static LimeLight limeLight;

	Command autonomousCommand;
	public static SendableChooser<Command> autonChooser;

	public static final ControlLooper controlLoop = new ControlLooper("Main control loop", Constants.kTimeoutMs);
	public static OI oi;

	@SuppressWarnings("unused")
	private boolean hasRun;

	public static enum OperationMode {
		TEST, COMPETITION
	};

	public static OperationMode operationMode = OperationMode.COMPETITION;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		driveTrain = DriveTrain.getInstance();
		intake = Intake.getInstance();
		arm = Arm.getInstance();
		elevator = Elevator.getInstance();
		oi = OI.getInstance();
		limeLight = LimeLight.getInstance();
		// controlLoop.addLoopable(driveTrain);
		// controlLoop.addLoopable(arm);
		// controlLoop.addLoopable(intake);
		comp = new Compressor();

		elevator.disEngageClimber();
		elevator.disEngageGravity();
		elevator.disEngageRope();
		// CameraServer.getInstance().startAutomaticCapture();
		elevator.elevatorLead.setSelectedSensorPosition(4000); // UNCOMMENT FOR MATCH
		arm.setStartConfigAngle(); // UNCOMMENT FOR MATCH

	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("SUCC Output Voltage", elevator.climbSUCC.getOutputCurrent());
		updateStatus();
	}

	@Override
	public void autonomousInit() {
		new IntakeSuction(IntakeState.SUCC_IN).start();
		elevator.elevatorLead.setSelectedSensorPosition(20700);
		arm.setStartConfigAngle();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		comp.start();
		limeLight.setLEID(LED.ON);
		Scheduler.getInstance().removeAll();
		// Robot.driveTrain.setControlMode(DriveTrainControlMode.JOYSTICK, 0);
		// arm.setControlMode(ArmControlMode.MANUAL);
		// driveTrain.setPeriodMs(Constants.kTimeoutMs);
		// controlLoop.start();
	}

	@Override
	public void teleopPeriodic() {
		updateStatus();
		Scheduler.getInstance().run();
	}

	public void disabledInit() {
		limeLight.setLEID(LED.OFF);
		// arm.resetArmEncoder();
		Scheduler.getInstance().removeAll();
		// intake.setSuction(Intake.IntakeState.SUCC_OUT);
	}

	public void disabledPeriodic() {
		updateStatus();
	}

	@Override
	public void testInit() {
		elevator.elevatorLead.setSelectedSensorPosition(4000); // UNCOMMENT FOR MATCH
		arm.setStartConfigAngle(); // UNCOMMENT FOR MATCH
	}

	@Override
	public void testPeriodic() {
		updateStatus();
		Scheduler.getInstance().run();

	}

	public void updateStatus() {
		arm.updateStatus(operationMode);
		driveTrain.updateStatus(operationMode);
		intake.updateStatus(operationMode);
		limeLight.postAllData();
	}

}
// !!!!!!!!!!!!see @link
// https://wpilib.screenstepslive.com/s/3120/m/7932/l/81109-choosing-an-autonomous-program-from-smartdashboard

// public void setupAutonChooser() {
// m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
// m_chooser.addOption("My Auto", kCustomAuto);
// SmartDashboard.putData("Auto choices", m_chooser);
// }