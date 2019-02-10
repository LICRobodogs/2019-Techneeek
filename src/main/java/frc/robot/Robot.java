/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import jaci.pathfinder.*;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Autonomous.Framework.AutoModeBase;
import frc.Autonomous.Framework.AutoModeExecuter;
import frc.Autonomous.Modes.BasicMode;
import frc.controller.Ps4_Controller;
import frc.robot.commands.GoStraightAtPercent;
import frc.robot.commands.JoystickDrive;
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
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class Robot extends TimedRobot { 
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
	private static JoystickDrive driveCommand;
	private static TurnToHeading turnCommand;
	private static GoStraightAtPercent straightPercent;
	private EncoderFollower m_left_follower;
   private EncoderFollower m_right_follower;
   private Notifier m_follower_notifier;

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
		driveCommand = new JoystickDrive();
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


  // @Override
  public void autonomousInit() {
		// autonomous();
		// turnCommand.start();
		driveBaseSubsystem.subsystemHome();
		// straightPercent.start();
		// turnCommand.start();
		//Path is in Feet/ probably not ideal
		SmartDashboard.putBoolean("\nStarted Auton",true);
		Trajectory left_trajectory = PathfinderFRC.getTrajectory("/home/lvuser/deploy/paths/Straight_Line.left.pf1.csv" + ".left");
		System.out.println(left_trajectory);
		SmartDashboard.putString("\nLeft trajectory",left_trajectory.toString());
     Trajectory right_trajectory = PathfinderFRC.getTrajectory("/home/lvuser/deploy/paths/Straight_Line.left.pf1.csv" + ".right");
     m_left_follower = new EncoderFollower(left_trajectory);
		 m_right_follower = new EncoderFollower(right_trajectory);
		 m_left_follower.configureEncoder((int)driveBaseSubsystem.getLeftDistanceInches(), Constants.DRIVE_TICKS_PER_ROTATION, Constants.kDriveWheelDiameterInches);
		 m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.kPathFollowingMaxVelSlow, 0);
		 m_right_follower.configureEncoder((int)driveBaseSubsystem.getRightDistanceInches(), Constants.DRIVE_TICKS_PER_ROTATION, Constants.kDriveWheelDiameterInches);
		 m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.kPathFollowingMaxVelSlow, 0);

		 m_follower_notifier = new Notifier(this::followPath);
     m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
	}
	private void followPath() {
		if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
			m_follower_notifier.stop();
		} else {
			double left_speed = m_left_follower.calculate(driveBaseSubsystem.getLeftPositionRaw());
			double right_speed = m_right_follower.calculate(driveBaseSubsystem.getRightPositionRaw());
			double heading = driveBaseSubsystem.getGyroAngle().getDegrees();
			double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
			double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
			double turn =  0.8 * (-1.0/80.0) * heading_difference;
			driveBaseSubsystem.setSpeed(left_speed + turn, right_speed - turn);
			
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
		m_follower_notifier.stop();
     driveBaseSubsystem.setSpeed(0, 0);
		driveBaseSubsystem.subsystemHome();
		
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
			// double move = oi.getMoveInput();
        // double steer = oi.getSteerInput();
        // driveBaseSubsystem.drive(move,steer);
    /*
    driveBaseSubsystem.m_drive.curvatureDrive(ps_controller.xSpeed(), ps_controller.zRotation(), true);
    if (ps_controller.isButtonPressed("SQUARE")) {
      System.out.println("Left Sensor Vel:" + driveBaseSubsystem.getLeftVelocityInchesPerSec());
      System.out.println("Sensor Pos:" + driveBaseSubsystem.getLeftDistanceInches());
      System.out.println("Angle is "+ driveBaseSubsystem.getGyroAngle());
      // System.out.println("Out %" + base.dt.leftDrive1.getMotorOutputPercent());
      // System.out.println("Out Of Phase:" + _faults.SensorOutOfPhase);
    }
    if (ps_controller.isButtonPressed("CIRCLE")) {
      System.out.println("Right Sensor Vel:" + driveBaseSubsystem.getRightVelocityInchesPerSec());
      System.out.println("Sensor Pos:" + driveBaseSubsystem.getRightDistanceInches());
      // System.out.println("Out %" + base.dt.leftDrive1.getMotorOutputPercent());
      // System.out.println("Out Of Phase:" + _faults.SensorOutOfPhase);
    }
    /*
    if (base.controller.getRawButton(2)){
      base.intake.getController().set(ControlMode.PercentOutput, -1);
      base.suctionSolenoid.set(true);
    }
    
    if (base.controller.getRawButton(4)){
      base.intake.getController().set(ControlMode.PercentOutput, 0);
      base.suctionSolenoid.set(false);
    }
    */
	}


  public void disabledInit() {
		// arm.resetArmEncoder();
		System.out.println("LEFT DISTANCE" + driveBaseSubsystem.getLeftDistanceInches());
		System.out.println("RIGHT DISTANCE" + driveBaseSubsystem.getRightDistanceInches());
		Scheduler.getInstance().removeAll();
	}

  @Override
  public void testPeriodic() {
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
