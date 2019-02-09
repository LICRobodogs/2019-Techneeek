/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Autonomous.Framework.AutoModeBase;
import frc.Autonomous.Framework.AutoModeExecuter;
import frc.Autonomous.Modes.BasicMode;
import frc.controller.Ps4_Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmControlMode;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainControlMode;
import frc.robot.subsystems.Intake;
import frc.util.ControlLooper;
import frc.util.CustomSubsystem;
import frc.util.ThreadRateControl;
import frc.util.drivers.Controllers;
import frc.util.loops.Looper;
import frc.util.loops.RobotStateEstimator;

public class Robot extends TimedRobot { 
  public static DriveTrain driveTrain;
  public static Intake intake;
	public static Arm arm;


	private Controllers robotControllers;
	
	private ArrayList<CustomSubsystem> subsystemVector; //use so we can instantiate everything in a forloop and then for every subsystem in here, register its loop  in the looper
	private Looper mLooper;
	private DriveBaseSubsystem driveBaseSubsystem;
	private RobotStateEstimator robotStateEstimator;
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	private AutoModeExecuter autoModeExecuter;
	private Ps4_Controller ps_controller;	

	//can delete below
	public DifferentialDrive myDrive;


	Command autonomousCommand;
	public static SendableChooser<Command> autonChooser;

	public static final ControlLooper controlLoop = new ControlLooper("Main control loop", 10);
	public static OI oi;

	@SuppressWarnings("unused")
	private boolean hasRun;

	public static enum OperationMode {
		TEST, COMPETITION
	};

	public static OperationMode operationMode = OperationMode.COMPETITION;

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
		mLooper = new Looper();
		ps_controller = new Ps4_Controller(0);
		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		driveBaseSubsystem.init();
		driveBaseSubsystem.registerEnabledLoops(mLooper); //we pass it the looper & it registers itself
		robotStateEstimator = RobotStateEstimator.getInstance();
		mLooper.register(robotStateEstimator); 

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
		//autonomousCommand.cancel();
		Scheduler.getInstance().removeAll();
		Robot.driveTrain.setControlMode(DriveTrainControlMode.JOYSTICK, 0);
		// arm.setControlMode(ArmControlMode.MANUAL);
		driveTrain.setPeriodMs(10);
		// controlLoop.start();
		myDrive = new DifferentialDrive(driveBaseSubsystem.getLeftMaster(), driveBaseSubsystem.getRightMaster());
	}

//   @Override
//   public void teleopPeriodic() {
//     updateStatus();
// 		Scheduler.getInstance().run();
//   }
  @Override
  public void teleopPeriodic() {
    
    myDrive.curvatureDrive(ps_controller.xSpeed(), ps_controller.zRotation(), true);
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
		Scheduler.getInstance().removeAll();
	}

  @Override
  public void testPeriodic() {
  }

  public void updateStatus() {
		arm.updateStatus(operationMode);
		driveTrain.updateStatus(operationMode);
	}
  
  public void setupAutonChooser() {
		autonChooser = new SendableChooser<>();
		// autonChooser.addDefault("Straight Only", new StraightOnly());
		autonChooser.addOption("Do Nothing", new CommandGroup());
		SmartDashboard.putData("Auton Setting", autonChooser);
	}
	public void autonomous() {
//		Controllers.getInstance().getCompressor().start();
//		Controllers.getInstance().getCompressor().setClosedLoopControl(true);

		mLooper.start(true);
		driveBaseSubsystem.setBrakeMode(true);
		autoModeExecuter = new AutoModeExecuter();

		
		AutoModeBase autoMode = new BasicMode();
		
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
			// ConsoleReporter.report(t, MessageLevel.ERROR);
		}
	}
}
