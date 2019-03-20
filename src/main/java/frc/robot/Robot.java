/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.LED;
import frc.util.Constants;
import frc.util.ControlLooper;

public class Robot extends TimedRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	private boolean m_LimelightHasValidTarget = false;
  	private double m_LimelightDriveCommand = 0.0;
  	private double m_LimelightSteerCommand = 0.0;

	private Spark pump;

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
		// intake = Intake.getInstance();
		// arm = Arm.getInstance();
		// elevator = Elevator.getInstance();
		oi = OI.getInstance();
		// CameraServer.getInstance().startAutomaticCapture();
		limeLight = LimeLight.getInstance();
		pump = new Spark(0);
		// controlLoop.addLoopable(driveTrain);
		// controlLoop.addLoopable(arm);
		// controlLoop.addLoopable(intake);
		// comp = new Compressor();
		// setupAutonChooser();
		// elevator.elevatorLead.setSelectedSensorPosition(4000);
		// arm.setStartConfigAngle();
	}

	@Override
	public void robotPeriodic() {
		// limeLight.postAllData();
		// updateStatus();
	}

	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// comp.start();
		// autonomousCommand.cancel();
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
		if(oi.getDriverGamepad().getAButton()){
			pump.set(0.4);
		}else{
			pump.set(0);
		}
	}

	public void disabledInit() {
		limeLight.setLEID(LED.OFF);

		// limeLight.setLEID(LED.OFF);
		// arm.resetArmEncoder();
		Scheduler.getInstance().removeAll();
		// intake.setSuction(Intake.IntakeState.SUCC_OUT);
	}

	public void disabledPeriodic() {
		updateStatus();

	}

	@Override
	public void testPeriodic() {
	}

	public void updateStatus() {
		// arm.updateStatus(operationMode);
		// driveTrain.updateStatus(operationMode);
		// intake.updateStatus(operationMode);
		// limeLight.getBasicData();
		limeLight.postAllData();
	}

	public void setupAutonChooser() {
		m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
	}
	
}
