/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Autonomous.Framework.AutoModeExecuter;
import frc.controller.Ps4_Controller;
import frc.robot.commands.GoStraightAtPercent;
import frc.robot.commands.JoyStickWithGyro;
import frc.robot.commands.TurnToHeading;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.Intake;
import frc.util.Constants;
import frc.util.CustomSubsystem;
import frc.util.ThreadRateControl;
import frc.util.drivers.Controllers;
import frc.util.loops.Looper;
import frc.util.loops.RobotStateEstimator;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;


public class Robot extends TimedRobot implements PIDOutput { 
  
  public static Intake intake;
	public static Arm arm;
	public static Controllers robotControllers;
	public static DriveBaseSubsystem driveBaseSubsystem;
	public static OI oi;
	private ArrayList<CustomSubsystem> subsystemVector; //use so we can instantiate everything in a forloop and then for every subsystem in here, register its loop  in the looper
	private Looper mLooper;
	
	private RobotStateEstimator robotStateEstimator;
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	private AutoModeExecuter autoModeExecuter;
	private static Ps4_Controller ps_controller;	
	private static Command driveCommand;
	private static TurnToHeading turnCommand;
	private static GoStraightAtPercent straightPercent;
	private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;
	private Notifier m_follower_notifier; 
	public PIDController turnController;
	private String pathToFollow = "Straight_Line";
 
	static final double kToleranceDegrees = 2.0f;
	double rotateToAngleRate;

	//can delete below
	public DifferentialDrive myDrive;
	Command autonomousCommand;
	public static SendableChooser<Command> autonChooser;

	@SuppressWarnings("unused")
	private boolean hasRun;

	public static enum OperationMode {
		TEST, COMPETITION
	};

	// public static OperationMode operationMode = OperationMode.TEST;
	
	 /* commented out for test build
  @Override
  public void robotInit() {
    driveTrain = new DriveTrain();
		intake = new Intake();
		arm = new Arm();
		oi = new OI();
		controlLoop.addLoopable(driveTrain);
		controlLoop.addLoopable(arm);
    controlLoop.addLoopable(intake);
    setupAutonChooser();
	}
	*/
	@Override
  public void robotInit() {
    robotControllers = Controllers.getInstance(); //want to spawn asap 
		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		robotStateEstimator = RobotStateEstimator.getInstance();
		driveCommand = new JoyStickWithGyro();
		turnCommand = new TurnToHeading(45);
		straightPercent = new GoStraightAtPercent(0.5);
		// mLooper = new Looper();
		// driveBaseSubsystem.registerEnabledLoops(mLooper); //we pass it the looper & it registers itself
		// mLooper.register(robotStateEstimator);
	}

  @Override
  public void robotPeriodic() {
	driveBaseSubsystem.dashUpdate();
	}
	
	public void startFileSearch(String name) {	
			String directory = Filesystem.getDeployDirectory().toString();
			findFile(name,new File(directory));
	}
	public void findFile(String name,File file) {
			File[] list = file.listFiles();
			if(list!=null)
			for (File fil : list)
			{
					if (fil.isDirectory())
					{
							findFile(name,fil);
					}
					else if (name.equalsIgnoreCase(fil.getName()))
					{
							System.out.println(fil.getParentFile());
					}
			}
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
	public File lFile() {
		String lFilePath = "/home/lvuser/deploy/output/"+pathToFollow+".left.pf1.csv";
		return new File(lFilePath);
	}
	public File rFile() {
		String rFilePath = "/home/lvuser/deploy/output/"+pathToFollow+".right.pf1.csv";
		return new File(rFilePath);
	}
  // @Override
  public void autonomousInit() {
		double kP = .1;
		double kd = kP/100; 

		SmartDashboard.putBoolean("\nStarted Auton",true);
		driveBaseSubsystem.subsystemHome();
	
		startCommand(turnCommand);
		startCommand(straightPercent);
		// startFileSearch();
		Trajectory left_trajectory = Pathfinder.readFromCSV(lFile());
		Trajectory right_trajectory = Pathfinder.readFromCSV(rFile());
		
		m_left_follower = new EncoderFollower(left_trajectory);
		m_right_follower = new EncoderFollower(right_trajectory);
	

		m_left_follower.configureEncoder((int)driveBaseSubsystem.getLeftDistanceInches(), Constants.DRIVE_TICKS_PER_ROTATION, Constants.kDriveWheelDiameterInches);
		m_right_follower.configureEncoder((int)driveBaseSubsystem.getRightDistanceInches(), Constants.DRIVE_TICKS_PER_ROTATION, Constants.kDriveWheelDiameterInches);
		 
		m_left_follower.configurePIDVA(kP, 0.0, kd, Constants.lKv, Constants.lKa);
		m_right_follower.configurePIDVA(kP, 0.0, kd, Constants.rKv, Constants.rKa);

		m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
	}

	private void followPath() {
		if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
			m_follower_notifier.stop();
		} else {
			double left_speed = m_left_follower.calculate(driveBaseSubsystem.getLeftPositionRaw());
			double right_speed = m_right_follower.calculate(driveBaseSubsystem.getRightPositionRaw());
			double heading = Controllers.getInstance().getGyro().getAngle();
			double desired_heading = -Pathfinder.r2d(m_left_follower.getHeading());
			double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);

			SmartDashboard.putNumber("Heading", heading);
			SmartDashboard.putNumber("Heading Desired", desired_heading);
			SmartDashboard.putNumber("Heading Difference", heading_difference);

        double kP = 0.005; // propotional turning constant
        double turningValue = (desired_heading - heading) * kP;
		// Invert the direction of the turn if we are going backwards
	
			double turn =  0.8 * (-1.0/80.0) * heading_difference;
			// driveBaseSubsystem.setSpeed(left_speed + turn, right_speed - turn);
			double daSpeed = (left_speed + right_speed)/2;
			 daSpeed = left_speed;
			// turningValue = Math.copySign(turningValue, Robot.getPsController().xSpeed());
			turningValue = Math.copySign(turningValue, daSpeed);
			
			driveBaseSubsystem.drive(daSpeed, turningValue);
			
		}
}

  @Override
  public void autonomousPeriodic() {
	driveBaseSubsystem.dashUpdate();
    Scheduler.getInstance().run();
	}


  @Override
	public void teleopInit() {
		driveBaseSubsystem.subsystemHome();
		m_follower_notifier.stop();
    driveBaseSubsystem.setSpeed(0, 0);
		driveCommand.start();
		Scheduler.getInstance().removeAll();
	}

  @Override
  public void teleopPeriodic() {
	driveBaseSubsystem.dashUpdate();
		Scheduler.getInstance().run();
		// turn();

	}
	
	public void turn() {
			boolean rotateToAngle = false;
			if ( ps_controller.isButtonPressed("X")) {
				robotControllers.getGyro().reset();
			}
			if ( ps_controller.isButtonPressed("X")) {
					turnController.setSetpoint(0.0f);
					rotateToAngle = true;
			} else if ( ps_controller.isButtonPressed("CIRCLE")) {
					turnController.setSetpoint(90.0f);
					rotateToAngle = true;
			} else if ( ps_controller.isButtonPressed("TRIANGLE")) {
					turnController.setSetpoint(179.9f);
					rotateToAngle = true;
			} else if ( ps_controller.isButtonPressed("SQUARE")) {
					turnController.setSetpoint(-90.0f);
					rotateToAngle = true;
			}
			double currentRotationRate;
			if ( rotateToAngle ) {
					turnController.enable();
					currentRotationRate = rotateToAngleRate;
			} else {
					turnController.disable();
					currentRotationRate = ps_controller.getTwist();
			}
			try {
					/* Use the joystick X axis for lateral movement,          */
					/* Y axis for forward movement, and the current           */
					/* calculated rotation rate (or joystick Z axis),         */
					/* depending upon whether "rotate to angle" is active.    */
					// driveBaseSubsystem.drive(ps_controller.getX(), ps_controller.getY(), 
					// 															 currentRotationRate, robotControllers.getGyro().getAngle());
			} catch( RuntimeException ex ) {
					DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
			}
			Timer.delay(0.005);		// wait for a motor update time
	}

@Override
  public void disabledInit() {
		driveBaseSubsystem.subsystemHome();
		System.out.println("LEFT DISTANCE" + driveBaseSubsystem.getLeftDistanceInches());
		System.out.println("RIGHT DISTANCE" + driveBaseSubsystem.getRightDistanceInches());
		Scheduler.getInstance().removeAll();
	}

  @Override
  public void testPeriodic() {
	}
	 /**
   * Runs during test mode
   */
  public void test() {
  }

  @Override
  /* This function is invoked periodically by the PID Controller, */
  /* based upon navX-MXP yaw angle input and PID Coefficients.    */
  public void pidWrite(double output) {
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
