/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controller.Ps4_Controller;
import frc.robot.PathFiles.paths;
import frc.robot.commands.GoStraightAtPercent;
import frc.robot.commands.JoyStickWithGyro;
import frc.robot.commands.StraightPath;
import frc.robot.commands.TurnToHeading;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.Intake;
import frc.util.drivers.Controllers;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import frc.util.Util;

public class Robot extends TimedRobot implements PIDOutput {
	public static Intake intake;
	public static Arm arm;
	public static Controllers robotControllers;
	public static DriveBaseSubsystem driveBaseSubsystem;
	public static OI oi;

	private static Ps4_Controller ps_controller;
	private static Command driveCommand;
	private static TurnToHeading turnCommand;
	private static GoStraightAtPercent straightPercent;

	public PIDController turnController;
	public PIDController driveController;
	private String pathToFollow = "Straight_Line";
	private double previous_error = 0.0;
	private double integral = 0.0;
	public static Trajectory left_trajectory;
	public static Trajectory right_trajectory;
	public static PathFiles trajectoryFiles;

	static final double kToleranceDegrees = 2.0f;
	double rotateToAngleRate;
	double driveToPositionRate;

	Command autonomousCommand;
	public static SendableChooser<Command> autonChooser;

	public static enum OperationMode {
		TEST, COMPETITION
	};

	// public static OperationMode operationMode = OperationMode.TEST;
	/*
	 * commented out for test build
	 * 
	 * @Override public void robotInit() { driveTrain = new DriveTrain(); intake =
	 * new Intake(); arm = new Arm(); oi = new OI();
	 * controlLoop.addLoopable(driveTrain); controlLoop.addLoopable(arm);
	 * controlLoop.addLoopable(intake); setupAutonChooser(); }
	 */
	@Override
	public void robotInit() {
		robotControllers = Controllers.getInstance(); // want to spawn asap
		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		
		trajectoryFiles = PathFiles.getInstance(); // this too, spawn asap

		driveCommand = new JoyStickWithGyro();
		turnCommand = new TurnToHeading(45);
		straightPercent = new GoStraightAtPercent(0.5);

		ps_controller = robotControllers.getPS_Controller();
		setUpTurnController();
		driveController = driveBaseSubsystem.getPIDController();
		// left_trajectory = Pathfinder.readFromCSV(Util.lFile("Straight_Line"));
		// right_trajectory = Pathfinder.readFromCSV(Util.rFile("Straight_Line"));

		// mLooper = new Looper();
		// driveBaseSubsystem.registerEnabledLoops(mLooper); //we pass it the looper &
		// it registers itself
		// mLooper.register(robotStateEstimator);
	}

	public void setUpTurnController() {
		double kP = 0.4, kI = 0.0, kD = 0.0, kF = 0.0;
		turnController = new PIDController(kP, kI, kD, kF, robotControllers.getGyro(), this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);

		/* Add the PID Controller to the Test-mode dashboard, allowing manual */
		/* tuning of the Turn Controller's P, I and D coefficients. */
		/* Typically, only the P value needs to be modified. */
		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
	}

	@Override
	public void robotPeriodic() {
		driveBaseSubsystem.dashUpdate();
	}

	public void startCommand(Command toStart) {
		toStart.start();
	}

	public void setPath(String path) {
		pathToFollow = path;
	}

	public String getPath(String path) {
		return pathToFollow;
	}

	// @Override
	public void autonomousInit() {

		setPath("Straight_Line");
		SmartDashboard.putBoolean("\nStarted Auton", true);
		driveBaseSubsystem.subsystemHome();
		clearSchedular();
		Command com = new StraightPath("Straight_Path");
		startCommand(com);
	}

	@Override
	public void autonomousPeriodic() {
		driveBaseSubsystem.dashUpdate();
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		driveBaseSubsystem.subsystemHome();
		// stopNotifier();
		driveBaseSubsystem.setSpeed(0, 0);
		clearSchedular();
		// startCommand(new JoyStickWithGyro());
		startCommand(new JoyStickWithGyro());
	}

	@Override
	public void teleopPeriodic() {
		driveBaseSubsystem.dashUpdate();
		Scheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		driveBaseSubsystem.subsystemHome();
		System.out.println("LEFT DISTANCE" + driveBaseSubsystem.getLeftDistanceInches());
		System.out.println("RIGHT DISTANCE" + driveBaseSubsystem.getRightDistanceInches());
		Scheduler.getInstance().removeAll();
		turnController.reset();
		driveController.reset();
	}

	@Override
	public void testPeriodic() {
		// turn();
		drive();
	}

	@Override
	public void testInit() {
		driveBaseSubsystem.subsystemHome();
		// stopNotifier();
		driveBaseSubsystem.setSpeed(0, 0);
		clearSchedular();
	}
	

	// public void stopNotifier() {
	// try {
	// m_follower_notifier.stop();
	// } catch (NullPointerException e) {
	// }
	// }
	public void clearSchedular() {
		try {
			Scheduler.getInstance().removeAll();
		} catch (NullPointerException e) {
		}
	}

	public void turn() {
		boolean rotateToAngle = false;
		if (ps_controller.isButtonPressed("LB")) {
			driveBaseSubsystem.subsystemHome();
			turnController.setSetpoint(0.0f);
			rotateToAngle = true;
		} else if (ps_controller.isButtonPressed("X")) {
			turnController.setSetpoint(0.0f);
			rotateToAngle = true;
		} else if (ps_controller.isButtonPressed("CIRCLE")) {
			turnController.setSetpoint(90.0f);
			rotateToAngle = true;
		} else if (ps_controller.isButtonPressed("TRIANGLE")) {
			turnController.setSetpoint(180.0f);
			rotateToAngle = true;
		} else if (ps_controller.isButtonPressed("SQUARE")) {
			turnController.setSetpoint(-90.0f);
			rotateToAngle = true;
		}
		double currentRotationRate;
		if (rotateToAngle) {
			turnController.enable();
			currentRotationRate = rotateToAngleRate;
		} else {
			turnController.disable();
			currentRotationRate = ps_controller.getTwist();
		}
		try {
			/* Use the joystick X axis for lateral movement, */
			/* Y axis for forward movement, and the current */
			/* calculated rotation rate (or joystick Z axis), */
			/* depending upon whether "rotate to angle" is active. */
			// double error = (turnController.getSetpoint() -
			// robotControllers.getGyro().getYaw());

			// double pro = turnController.getError() * turnController.getP();
			// double deriv = (turnController.getError() - this.previous_error) / .02;
			// this.integral += (turnController.getError()*.02);
			// driveBaseSubsystem.steer((pro + integral*turnController.getI() +
			// deriv*turnController.getD()));
			// previous_error = turnController.getError();

			double error = (turnController.getSetpoint() - robotControllers.getGyro().getYaw());
			double scaled = Math.pow((error * turnController.getP())/180, 2);
			driveBaseSubsystem.steer(scaled);
			SmartDashboard.putNumber("Error",error);

			// currentRotationRate, robotControllers.getGyro().getAngle());
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
		}
		Timer.delay(0.005); // wait for a motor update time
	}

	public void drive() {
		boolean rotateToAngle = false;
		if (ps_controller.isButtonPressed("LB")) {
			driveBaseSubsystem.subsystemHome();
			driveController.setSetpoint(0.0);
		}
		else if (ps_controller.isButtonPressed("X")) {
			// driveController.setSetpoint(driveBaseSubsystem.inchesToSensorPosition(2.0));
			// driveController.setSetpoint(driveBaseSubsystem.inchesToSensorPosition(6.0));
			driveController.setSetpoint(6.0);
			rotateToAngle = true;
		} else if (ps_controller.isButtonPressed("CIRCLE")) {
			driveController.setSetpoint(-6.0);
			rotateToAngle = true;
		}
		// else if ( ps_controller.isButtonPressed("TRIANGLE")) {
		// turnController.setSetpoint(180.0f);
		// rotateToAngle = true;
		// } else if ( ps_controller.isButtonPressed("SQUARE")) {
		// turnController.setSetpoint(-90.0f);
		// rotateToAngle = true;
		// }
		double positionRate;
		if (rotateToAngle) {
			driveController.enable();
			positionRate = driveToPositionRate;
		} else {
			driveController.disable();
			positionRate = ps_controller.getTwist();
		}
		try {
			/* Use the joystick X axis for lateral movement, */
			/* Y axis for forward movement, and the current */
			/* calculated rotation rate (or joystick Z axis), */
			/* depending upon whether "rotate to angle" is active. */
			// double turningValue = (turnController.getSetpoint() -
			// robotControllers.getGyro().getYaw()) * 0.08;
			// double driveValue = (driveController.getSetpoint() - driveBaseSubsystem.getLeftPositionRaw())
			// 		* driveController.getP();
			// double scaled = driveValue/driveController.getSetpoint();
			// driveBaseSubsystem.drive(scaled, 0);
			SmartDashboard.putNumber("Error",driveController.getError());

			// currentRotationRate, robotControllers.getGyro().getAngle());
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
		}
		Timer.delay(0.005); // wait for a motor update time
	}

	/**
	 * Runs during test mode
	 */
	public void test() {
	}

	@Override
	/* This function is invoked periodically by the PID Controller, */
	/* based upon navX-MXP yaw angle input and PID Coefficients. */
	public void pidWrite(double output) {
		driveToPositionRate = output;
		// driveBaseSubsystem.steer(output);
		rotateToAngleRate = output;
	}


	public void setupAutonChooser() {
		autonChooser = new SendableChooser<>();
		// autonChooser.addDefault("Straight Only", new StraightOnly());
		// autonChooser.addDefault("Straight Only", new BasicMode());
		autonChooser.addOption("Do Nothing", new CommandGroup());
		SmartDashboard.putData("Auton Setting", autonChooser);
	}
}
