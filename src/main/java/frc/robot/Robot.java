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
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainControlMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.util.Constants;
import frc.util.ControlLooper;

public class Robot extends TimedRobot {
	public static DriveTrain driveTrain;
	public static Intake intake;
	public static Arm arm;
	public static Elevator elevator;
	public static Compressor comp;

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
		// controlLoop.addLoopable(driveTrain);
		// controlLoop.addLoopable(arm);
		// controlLoop.addLoopable(intake);
		comp = new Compressor();
		setupAutonChooser();
		
		elevator.elevatorLead.setSelectedSensorPosition(4000);
		arm.setStartConfigAngle();
	}

	@Override
	public void robotPeriodic() {
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		comp.stop();
		// autonomousCommand.cancel();
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
		arm.updateStatus(operationMode);
		// driveTrain.updateStatus(operationMode);
	}

	public void setupAutonChooser() {
		autonChooser = new SendableChooser<>();
		// autonChooser.addDefault("Straight Only", new StraightOnly());
		autonChooser.addOption("Do Nothing", new CommandGroup());
		SmartDashboard.putData("Auton Setting", autonChooser);
	}
}
