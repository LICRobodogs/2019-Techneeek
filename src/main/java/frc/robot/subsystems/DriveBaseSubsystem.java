package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Kinematics;
import frc.robot.commands.*;
import frc.util.Constants;
import frc.util.Util;
import frc.util.TrajectoryFollowingMotion.Lookahead;
import frc.util.TrajectoryFollowingMotion.Path;
import frc.util.TrajectoryFollowingMotion.PathFollower;
import frc.util.TrajectoryFollowingMotion.PathFollowerRobotState;
import frc.util.drivers.Controllers;
import frc.util.drivers.DriveControlState;
import frc.util.drivers.DriveMotorValues;
import frc.util.drivers.DunkTalonSRX;
import frc.util.drivers.DunkVictorSPX;
import frc.util.loops.Loop;
import frc.util.math.RigidTransform2d;
import frc.util.math.Twist2d;



public class DriveBaseSubsystem extends Subsystem implements PIDOutput {
     private static DriveBaseSubsystem instance = null;
     private DunkTalonSRX leftMaster, leftSlave, rightMaster;
     private DunkVictorSPX rightSlave;
     // private DunkGyro gyro;
     // private AHRS gyro;
     private DriveControlState controlMode;
     private static ReentrantLock _subsystemMutex = new ReentrantLock();
     private boolean previousBrakeModeVal;

     private Path mCurrentPath = null;
     private PathFollower mPathFollower;
     private PathFollowerRobotState robotState = PathFollowerRobotState.getInstance();
     // public static final ADIS16470_IMU imu = new ADIS16470_IMU();
     public DifferentialDrive m_drive;

     public PIDController turnController;
     private final double kp = 0.0;
     private final double ki = 0.0;
     private final double kd = 0.0;
     private final double kf = 0.0;
     private Controllers robotControllers;
     public static DriveBaseSubsystem getInstance() {
          if (instance == null) 
               instance = new DriveBaseSubsystem();
          return instance;
     }

     private DriveBaseSubsystem() {
          
          robotControllers = Controllers.getInstance();
          leftMaster = robotControllers.getLeftDrive1();
          leftSlave = robotControllers.getLeftDrive2();
          rightMaster = robotControllers.getRightDrive1();
          rightSlave = robotControllers.getRightDrive2();
          // leftSlave.setInverted(true);
          // leftMaster.setInverted(true);
          leftSlave.follow(leftMaster);
          rightSlave.follow(rightMaster);
          leftMaster.setSensorPhase(true);
          // rightMaster.setSensorPhase(true);
          m_drive = new DifferentialDrive(leftMaster, rightMaster);
          m_drive.setSafetyEnabled(false);
          
          // gyro = robotControllers.getGyro();
          
          previousBrakeModeVal = false;
          // setBrakeMode(true);

          // controlMode = DriveControlState.PATH_FOLLOWING; //because we start match on auton
          turnController = new PIDController(kp,ki,kd,robotControllers.getGyro(),this);
          turnController.setInputRange(-180,180);
          turnController.setOutputRange(-0.45,0.45);
          turnController.setAbsoluteTolerance(2.0f);
          turnController.setContinuous(true);

     }

     private final Loop mLoop = new Loop() {
          
          
          public void onFirstStart(double timestamp) {
               synchronized(DriveBaseSubsystem.this) {
                    subsystemHome();
               }
          }
          @Override
          public void onStart(double timestamp) {
               synchronized(DriveBaseSubsystem.this) {
                    setDriveOpenLoop(DriveMotorValues.NEUTRAL);
                    setBrakeMode(false);
                    setDriveVelocity(new DriveMotorValues(0,0));
               }
          }
          @Override
          public void onLoop(double timestamp, boolean isAuto) {
               synchronized(DriveBaseSubsystem.this) {
                    switch(controlMode) {
                         case OPEN_LOOP:
                              break;
                         case VELOCITY:
                              break;
                         case TURN_TO_HEADING:
                              break;
                         case PATH_FOLLOWING:
                              break;
                         default:
                              System.err.print("Unexpected Control Mode");
                    }
               }
          }
          @Override
          public void onStop(double timestamp){
               synchronized(DriveBaseSubsystem.this) {
                    setDriveOpenLoop(DriveMotorValues.NEUTRAL);
               }
          }
     };
     public void drive(double move, double steer) {
		m_drive.curvatureDrive(move, steer, true);
	}
     	
	public void rotateDegrees(double angle) {
		robotControllers.getGyro().reset();
		turnController.reset();
		turnController.setPID(kp, ki, kd);
		turnController.setSetpoint(angle);
		turnController.enable();
	}
     public void subsystemHome() {
          robotControllers.getGyro().reset();
          boolean setSucceeded;
          int retryCounter = 0;
          do {
               setSucceeded = true;
               setSucceeded &= leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kMSTimeoutsFast) == ErrorCode.OK;
               setSucceeded &= rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kMSTimeoutsFast) == ErrorCode.OK;
          } while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

     }
     public synchronized void setDriveOpenLoop(DriveMotorValues d) {
          setControlMode(DriveControlState.OPEN_LOOP);

          leftMaster.set(ControlMode.PercentOutput, d.leftDrive);
          rightMaster.set(ControlMode.PercentOutput, d.rightDrive);
     }
     public void setControlMode(DriveControlState controlMode) {
          if (this.controlMode != controlMode) {
               try {
                    _subsystemMutex.lock();
                    this.controlMode = controlMode;
                    
               } catch(Exception e) {
                    System.err.println("Error with setting drice control mode");
               } finally {
                    _subsystemMutex.unlock();
               }
               
          }

     }
     public void setSpeed(double speed1, double speed2) {
		// if (speed == 0) {
		/*
		 * } else { setControlMode(DriveTrainControlMode.TEST); rightDrive1.set(speed);
		 * leftDrive1.set(speed); }
		 */
          
		leftMaster.set(speed1);
		rightMaster.set(speed2);
	}
     public void setPercentSpeed(double percent) {
		setPercentSpeed(percent, percent);
	}
     public void setPercentSpeed(double percentL, double percentR) {
		leftMaster.set(ControlMode.PercentOutput, percentL);
		rightMaster.set(ControlMode.PercentOutput, percentR);
	}
     public void setBrakeMode(boolean brakeMode){
          if (previousBrakeModeVal != brakeMode) {
               _subsystemMutex.lock();
               leftMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
               leftSlave.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
               rightMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
               rightSlave.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
               previousBrakeModeVal = brakeMode;
               _subsystemMutex.unlock();
          }
     }
     public synchronized void setDriveVelocity(DriveMotorValues d) {
          setDriveVelocity(d, true);
     }
     public synchronized void setDriveVelocity(DriveMotorValues d, boolean autoChangeMode) {
          if (autoChangeMode)
               setControlMode(DriveControlState.VELOCITY);
          leftMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(d.leftDrive));
          rightMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(d.rightDrive));
     }
     /**
	 * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
	 * pose, distance driven, and velocity, then updates the wheel velocity setpoints.
	 */
	private void updatePathFollower(double timestamp) {
		RigidTransform2d robot_pose = robotState.getLatestFieldToVehicle().getValue();

          Twist2d command = mPathFollower.update(timestamp, robot_pose, PathFollowerRobotState.getInstance().getDistanceDriven(), PathFollowerRobotState.getInstance().getPredictedVelocity().dx);
		if (!mPathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			updatePathVelocitySetpoint(setpoint.left, setpoint.right);

			System.out.println(mPathFollower.getDebug());
			System.out.println("Left2Cube: " + inchesPerSecondToRpm(setpoint.left) + ", Right2Cube: " + inchesPerSecondToRpm(setpoint.right));
			System.out.println("Left2Cube Actual: " + Util.convertNativeUnitsToRPM(leftMaster.getSelectedSensorVelocity(0)) + ", Right2Cube Actual: " + Util.convertNativeUnitsToRPM(rightMaster.getSelectedSensorVelocity(0)));
		} else {
			updatePathVelocitySetpoint(0, 0);
			System.out.println("Completed path!");
			setControlMode(DriveControlState.VELOCITY);
		}
     }
     private void updatePathVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
		final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;

		leftMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(inchesPerSecondToRpm(left_inches_per_sec * scale)));
		rightMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(inchesPerSecondToRpm(right_inches_per_sec * scale)));

		System.out.println("Requested Drive Velocity Left2Cube/Right2Cube: " + left_inches_per_sec + "/" + right_inches_per_sec);
		System.out.println("Actual Drive Velocity Left2Cube/Right2Cube: " + getLeftVelocityInchesPerSec() + "/" + getRightVelocityInchesPerSec());
     }
     private static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60;
     }
     private static double inchesToRotations(double inches) {
		return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
     }
     private static double rotationsToInches(double rotations) {
		return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
     }
     private static double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60;
	}
     public double getLeftDistanceInches() {
		return rotationsToInches(leftMaster.getSelectedSensorPosition(0)/Constants.kSensorUnitsPerRotation);
	}

	public double getRightDistanceInches() {
		return rotationsToInches(rightMaster.getSelectedSensorPosition(0)/Constants.kSensorUnitsPerRotation);
     }
     public int getRightPositionRaw() {
          return rightMaster.getSelectedSensorPosition(0);
     }
     
     public int getLeftPositionRaw() {
          return leftMaster.getSelectedSensorPosition(0);
     }

	public double getLeftVelocityInchesPerSec() { return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(leftMaster.getSelectedSensorVelocity(0))); }

	public double getRightVelocityInchesPerSec() { return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(rightMaster.getSelectedSensorVelocity(0))); }
    
     public double getGyroAngle() {
          return robotControllers.getGyro().getAngle();
     }


     // public synchronized void setGyroAngle(double angle) {
	// 	robotControllers.getGyro().reset();
	// 	robotControllers.getGyro().setAngleAdjustment(angle);
     // }
     public DunkTalonSRX getLeftMaster() {
          return leftMaster;
     }
     public DunkTalonSRX getRightMaster() {
          return rightMaster;
     }
     public void dashUpdate() {
          
          // SmartDashboard.putNumber("\nLeft Velocity (in/sec)",getLeftVelocityInchesPerSec());
          // SmartDashboard.putNumber("\nRight Velocity (in/sec)",getRightVelocityInchesPerSec());
          // SmartDashboard.putNumber("\nGyro Angle in Degrees",getGyroAngle().getDegrees());
          SmartDashboard.putNumber("\nGyro Yaw Angle in Degrees",getGyroAngle());
          SmartDashboard.putNumber("\nLeft Distance (in)",getLeftDistanceInches());
          SmartDashboard.putNumber("\nRight Distance (in)",getRightDistanceInches());
          
          

     }
     /**
	 * Configures the drivebase to drive a path. Used for autonomous driving
	 *
	 * @see Path
	 */
	public synchronized void setWantDrivePath(Path path, boolean reversed) {
		if (mCurrentPath != path || controlMode != DriveControlState.PATH_FOLLOWING) {
			setControlMode(DriveControlState.PATH_FOLLOWING);
			PathFollowerRobotState.getInstance().resetDistanceDriven();
			mPathFollower = new PathFollower(path, reversed,
					new PathFollower.Parameters(
							new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
									Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
							Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
							Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
							Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
							Constants.kPathFollowingMaxVelSlow, Constants.kPathFollowingMaxAccel,
							Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
							Constants.kPathStopSteeringDistance));

			mCurrentPath = path;
		} else {

			System.err.println("Error setting path for drive!"); 
		}
     }
     public synchronized boolean isDoneWithPath() {
		if (controlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			System.out.println("Robot has completed the path");
			return mPathFollower.isFinished();
		} else {
			System.out.println("Robot is not in path following mode");
			if (mPathFollower != null)
				return mPathFollower.isFinished();
			else
				return true;
		}
     }
     public synchronized void forceDoneWithPath() {
		if (controlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			System.out.println("Forcing robot to stop the path");
			mPathFollower.forceFinish();
		} else {
			System.out.println("Robot is not in path following mode");
		}
	}

	public synchronized boolean hasPassedMarker(String marker) {
		if (controlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			System.out.println("Robot has passed marker " + marker);
			return mPathFollower.hasPassedMarker(marker);
		} else {
			System.out.println("Robot is not in path following mode");
			if (mPathFollower != null)
				return (mPathFollower.isFinished() || mPathFollower.hasPassedMarker(marker));
			else {
				//TODO: Test with false value
				return true;
			}
		}
     }
     
     @Override
     protected void initDefaultCommand() {
         setDefaultCommand(new JoystickDrive());
     }

	@Override
	public void pidWrite(double output) {
          setControlMode(DriveControlState.OPEN_LOOP);
          leftMaster.set(ControlMode.PercentOutput, -output);
          rightMaster.set(ControlMode.PercentOutput, output);
	}

     // public void init() {
     //      //this is where we invert and set phase of motors if needed
     //      setBrakeMode(true);

     //      boolean setSucceeded;
	// 	int retryCounter = 0;

	// 	do {
	// 		setSucceeded = true;

	// 		setSucceeded &= leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
	// 		setSucceeded &= leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
	// 		setSucceeded &= leftMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

	// 		setSucceeded &= rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
	// 		setSucceeded &= rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
	// 		setSucceeded &= rightMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

	// 	} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

     // //TalonHelper is just so you don't have to type out config ki, config kp, config kd that the talon library uses
     // //Not shifting so assign same gear
     // //Set same gains and use slots 0 & 1 
     // // Make sure we set the gains on both sides - left and irght - use slots 0 & 1 just sl that everything has the same gains and we don't get confused betweeen gain scheduling slots in the talons
	// 	// setSucceeded &= TalonHelper.setPIDGains(leftMaster, 0, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
	// 	// setSucceeded &= TalonHelper.setPIDGains(rightMaster, 0, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
	// 	// setSucceeded &= TalonHelper.setPIDGains(rightMaster, 1, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
	// 	// setSucceeded &= TalonHelper.setPIDGains(leftMaster, 1, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);

	// 	leftMaster.selectProfileSlot(0, 0);
	// 	rightMaster.selectProfileSlot(0, 0);
     // }
}