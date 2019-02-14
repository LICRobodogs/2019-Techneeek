/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Autonomous.Framework.AutoModeBase;
import frc.Autonomous.Framework.AutoModeExecuter;
import frc.Autonomous.Modes.BasicMode;
import frc.controller.Ps4_Controller;
import frc.robot.commands.GoStraightAtPercent;
import frc.robot.commands.*;
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
  // public static DriveTrain driveTrain;
  public static Intake intake;
	public static Arm arm;


	private Controllers robotControllers;
	
	private ArrayList<CustomSubsystem> subsystemVector; //use so we can instantiate everything in a forloop and then for every subsystem in here, register its loop  in the looper
	private Looper mLooper;
	public static DriveBaseSubsystem driveBaseSubsystem;
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
 
		 static final double kToleranceDegrees = 2.0f;
		 double rotateToAngleRate;

	//can delete below
	public DifferentialDrive myDrive;


	Command autonomousCommand;
	public static SendableChooser<Command> autonChooser;

	// public static final ControlLooper controlLoop = new ControlLooper("Main control loop", 10);
	public static OI oi;

	@SuppressWarnings("unused")
	private boolean hasRun;

	public static enum OperationMode {
		TEST, COMPETITION
	};

	// public static OperationMode operationMode = OperationMode.TEST;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
	
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
		// mLooper = new Looper();
		ps_controller = new Ps4_Controller(0);
		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		// driveBaseSubsystem.init();
		// driveBaseSubsystem.registerEnabledLoops(mLooper); //we pass it the looper & it registers itself
		driveBaseSubsystem.subsystemHome();
		robotStateEstimator = RobotStateEstimator.getInstance();
		// driveCommand = new JoystickDrive();
		driveCommand = new JoyStickWithGyro();
		// turnCommand = new TurnToHeading(45);
		straightPercent = new GoStraightAtPercent(0.5);
		// mLooper.register(robotStateEstimator); 


	}
	public static Ps4_Controller getPsController() {
		return ps_controller;
	}
  @Override
  public void robotPeriodic() {
	driveBaseSubsystem.dashUpdate();
  }
	public void startFileSearch() {	
			String name = "Straight_Line.left.pf1.csv";
			String directory = Filesystem.getDeployDirectory().toString();
			findFile(name,new File(directory));
	}
	public void findFile(String name,File file)
	{
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

  // @Override
  public void autonomousInit() {
		// autonomous();
		// turnCommand.start();
		System.out.println("Starting Search");
		startFileSearch();
		System.out.println("Ending Search");
		driveBaseSubsystem.subsystemHome();
	
		// straightPercent.start();
		// turnCommand.start();
		//Path is in Feet/ probably not ideal
		SmartDashboard.putBoolean("\nStarted Auton",true);
		// SmartDashboard.putString("\nDeploy Directory",Filesystem.getDeployDirectory().toString());
		
		String name = "Straight_Line";
		// String lFilePath = Filesystem.getDeployDirectory().toString()+"/paths/"+name+".left.pdf1.csv";
		String lFilePath = "/home/lvuser/deploy/paths/Straight_Line.left.pf1.csv";
		String rFilePath = "/home/lvuser/deploy/paths/Straight_Line.right.pf1.csv";
		SmartDashboard.putString("\nLeft File Path",lFilePath);

		File myLFile = new File(lFilePath);
		Trajectory left_trajectory = Pathfinder.readFromCSV(myLFile);
		File myRFile = new File(rFilePath);
		Trajectory right_trajectory = Pathfinder.readFromCSV(myRFile);

		// Trajectory left_trajectory = PathfinderFRC.getTrajectory(Filesystem.getDeployDirectory().toString()+ "/Paths/" + name + ".left.pf1.csv");
		// Trajectory left_trajectory = PathfinderFRC.getTrajectory("/home/lvuser/paths/Straight_Line.left.pf1.csv");
		// System.out.println(left_trajectory);
		SmartDashboard.putBoolean("\nLeft trajectory loaded",true);
    //  Trajectory right_trajectory = PathfinderFRC.getTrajectory("/home/lvuser/paths/Straight_Line.left.pf1.csv");
     m_left_follower = new EncoderFollower(left_trajectory);
		 m_right_follower = new EncoderFollower(right_trajectory);
		//  kv= 0.5369 ka= 0.0889 in feet per whatever left
		// kv= 0.5158 ka= 0.0735 right
		double lKv = (0.5369/12);
		// double lKv = (0.5369);
		double lKa = (0.0889/12);
		// double lKa = (0.0889);
		double rKv = (0.5369/12);
		// double rKv = (0.5369);
		double rKa = (0.0889/12);
		// double rKa = (0.0889);
		double kP = .1;
		double kd = kP/100;
		 m_left_follower.configureEncoder((int)driveBaseSubsystem.getLeftDistanceInches(), Constants.DRIVE_TICKS_PER_ROTATION, Constants.kDriveWheelDiameterInches);
		 m_left_follower.configurePIDVA(kP, 0.0, kd, lKv, lKa);
		 m_right_follower.configureEncoder((int)driveBaseSubsystem.getRightDistanceInches(), Constants.DRIVE_TICKS_PER_ROTATION, Constants.kDriveWheelDiameterInches);
		 m_right_follower.configurePIDVA(kP, 0.0, kd, rKv, rKa);

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
			double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
			double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);

			SmartDashboard.putNumber("Heading", heading);
			SmartDashboard.putNumber("Heading Desired", desired_heading);
			SmartDashboard.putNumber("Heading Difference", heading_difference);

			double turn =  0.8 * (-1.0/80.0) * heading_difference;
			// driveBaseSubsystem.setSpeed(left_speed + turn, right_speed - turn);
			
			driveBaseSubsystem.drive(-(left_speed + right_speed)/2, turn);
			
		}
}

  @Override
  public void autonomousPeriodic() {
	driveBaseSubsystem.dashUpdate();
    // Scheduler.getInstance().run();
	}


//   @Override
// 	public void teleopInit() {
// 		//autonomousCommand.cancel();
// 		Scheduler.getInstance().removeAll();
// 		Robot.driveTrain.setControlMode(DriveTrainControlMode.JOYSTICK, 0);
// 		arm.setControlMode(ArmControlMode.MANUAL);
// 		driveTrain.setPeriodMs(10);
// 		controlLoop.start();
// 	}

  @Override
	public void teleopInit() {
		driveBaseSubsystem.subsystemHome();
		// robotControllers.getGyro().reset();
		// m_follower_notifier.stop();
     driveBaseSubsystem.setSpeed(0, 0);
		// driveBaseSubsystem.subsystemHome();
		
		driveCommand.start();


		// autonomousCommand.cancel();
		// Scheduler.getInstance().removeAll();
		// Robot.driveTrain.setControlMode(DriveTrainControlMode.JOYSTICK, 0);
		// arm.setControlMode(ArmControlMode.MANUAL);
		// driveTrain.setPeriodMs(10);
		// controlLoop.start();
		// myDrive = new DifferentialDrive(driveBaseSubsystem.getLeftMaster(), driveBaseSubsystem.getRightMaster());
	}

//   @Override
//   public void teleopPeriodic() {
//     updateStatus();
// 		Scheduler.getInstance().run();
//   }
  @Override
  public void teleopPeriodic() {
	driveBaseSubsystem.dashUpdate();
		Scheduler.getInstance().run();
		// turn();

	}
	
	// public void turn() {
	// 		boolean rotateToAngle = false;
	// 		if ( ps_controller.isButtonPressed("X")) {
	// 			robotControllers.getGyro().reset();
	// 		}
	// 		if ( ps_controller.isButtonPressed("X")) {
	// 				turnController.setSetpoint(0.0f);
	// 				rotateToAngle = true;
	// 		} else if ( ps_controller.isButtonPressed("CIRCLE")) {
	// 				turnController.setSetpoint(90.0f);
	// 				rotateToAngle = true;
	// 		} else if ( ps_controller.isButtonPressed("TRIANGLE")) {
	// 				turnController.setSetpoint(179.9f);
	// 				rotateToAngle = true;
	// 		} else if ( ps_controller.isButtonPressed("SQUARE")) {
	// 				turnController.setSetpoint(-90.0f);
	// 				rotateToAngle = true;
	// 		}
	// 		double currentRotationRate;
	// 		if ( rotateToAngle ) {
	// 				turnController.enable();
	// 				currentRotationRate = rotateToAngleRate;
	// 		} else {
	// 				turnController.disable();
	// 				currentRotationRate = ps_controller.getTwist();
	// 		}
	// 		try {
	// 				/* Use the joystick X axis for lateral movement,          */
	// 				/* Y axis for forward movement, and the current           */
	// 				/* calculated rotation rate (or joystick Z axis),         */
	// 				/* depending upon whether "rotate to angle" is active.    */
	// 				// driveBaseSubsystem.drive(ps_controller.getX(), ps_controller.getY(), 
	// 				// 															 currentRotationRate, robotControllers.getGyro().getAngle());
	// 		} catch( RuntimeException ex ) {
	// 				DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
	// 		}
	// 		Timer.delay(0.005);		// wait for a motor update time
	// }

@Override
  public void disabledInit() {
		driveBaseSubsystem.subsystemHome();
		SmartDashboard.putBoolean("\nStarted Auton",true);

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

  public void updateStatus() {
		// arm.updateStatus(operationMode);
		// driveTrain.updateStatus(operationMode);
	}

  
  public void setupAutonChooser() {
		autonChooser = new SendableChooser<>();
		// autonChooser.addDefault("Straight Only", new StraightOnly());
		// autonChooser.addDefault("Straight Only", new BasicMode());
		autonChooser.addOption("Do Nothing", new CommandGroup());
		SmartDashboard.putData("Auton Setting", autonChooser);
	}
	public void autonomous() {
//		Controllers.getInstance().getCompressor().start();
//		Controllers.getInstance().getCompressor().setClosedLoopControl(true);

		mLooper.start(true);
		driveBaseSubsystem.setBrakeMode(true);
		autoModeExecuter = new AutoModeExecuter();

		System.out.println("Pre basic mode");
		AutoModeBase autoMode = new BasicMode();
		System.out.println("Post basic mode");
		if (autoMode != null)
			autoModeExecuter.setAutoMode(autoMode);
		else
			return;

		autoModeExecuter.start();
		threadRateControl.start(true);

		while (isAutonomous() && isEnabled()) {threadRateControl.doRateControl(100);}
	}

	
	public void disabled() {
		exitAuto();//need to call bc 1 of biggest problems is auton ends & robot stuck in pathfinding mode & teleop can't control
		//force robot to exit auto and into teleop
		mLooper.stop();
		threadRateControl.start(true); //threadrate control is utility to help with loop timing so you don't have to do try catch everytime

		while (isDisabled()) {
			driveBaseSubsystem.setBrakeMode(false);
			threadRateControl.doRateControl(100);
		}

	}

	private void exitAuto() {
		try {
			if (autoModeExecuter != null) //automode executer is the thing that actually runs our autonmode
				autoModeExecuter.stop();

			// FileReporter.getInstance().terminate();

			autoModeExecuter = null;
		} catch (Throwable t) {
			System.err.println("Error"+t);
		}
	}
}
