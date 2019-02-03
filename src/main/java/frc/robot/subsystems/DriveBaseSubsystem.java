package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import frc.robot.Kinematics;
import frc.util.Constants;
import frc.util.CustomSubsystem;
import frc.util.Util;
import frc.util.TrajectoryFollowingMotion.Lookahead;
import frc.util.TrajectoryFollowingMotion.Path;
import frc.util.TrajectoryFollowingMotion.PathFollower;
import frc.util.TrajectoryFollowingMotion.PathFollowerRobotState;
import frc.util.drivers.Controllers;
import frc.util.drivers.DriveControlState;
import frc.util.drivers.DriveMotorValues;
import frc.util.drivers.DunkGyro;
import frc.util.drivers.DunkTalonSRX;
import frc.util.drivers.DunkVictorSPX;
import frc.util.drivers.TalonHelper;
import frc.util.loops.Loop;
import frc.util.loops.Looper;
import frc.util.math.RigidTransform2d;
import frc.util.math.Rotation2d;
import frc.util.math.Twist2d;

public class DriveBaseSubsystem implements CustomSubsystem {
     private static DriveBaseSubsystem instance = null;
     private DunkTalonSRX leftMaster, leftSlave, rightMaster;
     private DunkVictorSPX rightSlave;
     private DunkGyro gyro;
     private DriveControlState controlMode;
     private static ReentrantLock _subsystemMutex = new ReentrantLock();
     private boolean previousBrakeModeVal;

     private Path mCurrentPath = null;
     private PathFollower mPathFollower;
     private PathFollowerRobotState robotState = PathFollowerRobotState.getInstance();

     public static DriveBaseSubsystem getInstance() {
          if (instance == null) 
               instance = new DriveBaseSubsystem();
          return instance;
     }

     private DriveBaseSubsystem() {
          Controllers robotControllers = Controllers.getInstance();
          leftMaster = robotControllers.getLeftDrive1();
          leftSlave = robotControllers.getLeftDrive2();
          rightMaster = robotControllers.getRightDrive1();
          rightSlave = robotControllers.getRightDrive2();
          leftSlave.follow(leftMaster);
          rightSlave.follow(rightMaster);
          
          gyro = robotControllers.getGyro();
          
          previousBrakeModeVal = false;
          // setBrakeMode(true);

          controlMode = DriveControlState.PATH_FOLLOWING; //because we start match on auton
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
     
     public void subsystemHome() {
          gyro.zeroYaw();
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
               } finally {
                    _subsystemMutex.unlock();
               }
               
          }

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

			//ConsoleReporter.report(mPathFollower.getDebug());
			//ConsoleReporter.report("Left2Cube: " + inchesPerSecondToRpm(setpoint.left) + ", Right2Cube: " + inchesPerSecondToRpm(setpoint.right));
			//ConsoleReporter.report("Left2Cube Actual: " + Util.convertNativeUnitsToRPM(mLeftMaster.getSelectedSensorVelocity(0)) + ", Right2Cube Actual: " + Util.convertNativeUnitsToRPM(mRightMaster.getSelectedSensorVelocity(0)));
		} else {
			updatePathVelocitySetpoint(0, 0);
			// ConsoleReporter.report("Completed path!");
			setControlMode(DriveControlState.VELOCITY);
		}
     }
     private void updatePathVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
		final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;

		leftMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(inchesPerSecondToRpm(left_inches_per_sec * scale)));
		rightMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(inchesPerSecondToRpm(right_inches_per_sec * scale)));

		//ConsoleReporter.report("Requested Drive Velocity Left2Cube/Right2Cube: " + left_inches_per_sec + "/" + right_inches_per_sec);
		//ConsoleReporter.report("Actual Drive Velocity Left2Cube/Right2Cube: " + getLeftVelocityInchesPerSec() + "/" + getRightVelocityInchesPerSec());
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

	public double getLeftVelocityInchesPerSec() { return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(leftMaster.getSelectedSensorVelocity(0))); }

	public double getRightVelocityInchesPerSec() { return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(rightMaster.getSelectedSensorVelocity(0))); }
    
     public Rotation2d getGyroAngle() {
          return gyro.getYaw();
     }
     public synchronized void setGyroAngle(Rotation2d angle) {
		gyro.reset();
		gyro.setAngleAdjustment(angle);
     }
     public DunkTalonSRX getLeftMaster() {
          return leftMaster;
     }
     public DunkTalonSRX getRightMaster() {
          return rightMaster;
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
							Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
							Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
							Constants.kPathStopSteeringDistance));

			mCurrentPath = path;
		} else {
			// ConsoleReporter.report("Error setting path for drive!", MessageLevel.ERROR);
			System.err.println("Error setting path for drive!"); 
		}
     }
     public synchronized boolean isDoneWithPath() {
		if (controlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			// ConsoleReporter.report("Robot has completed the path");
			return mPathFollower.isFinished();
		} else {
			// ConsoleReporter.report("Robot is not in path following mode");
			if (mPathFollower != null)
				return mPathFollower.isFinished();
			else
				return true;
		}
     }
     public synchronized void forceDoneWithPath() {
		if (controlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			// ConsoleReporter.report("Forcing robot to stop the path");
			mPathFollower.forceFinish();
		} else {
			// ConsoleReporter.report("Robot is not in path following mode");
		}
	}

	public synchronized boolean hasPassedMarker(String marker) {
		if (controlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			// ConsoleReporter.report("Robot has passed marker " + marker);
			return mPathFollower.hasPassedMarker(marker);
		} else {
			// ConsoleReporter.report("Robot is not in path following mode");
			if (mPathFollower != null)
				return (mPathFollower.isFinished() || mPathFollower.hasPassedMarker(marker));
			else {
				//TODO: Test with false value
				return true;
			}
		}
	}



     @Override
     public void init() {
          //this is where we invert and set phase of motors if needed
          setBrakeMode(true);

          boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= leftMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= rightMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

     //TalonHelper is just so you don't have to type out config ki, config kp, config kd that the talon library uses
     //Not shifting so assign same gear
     //Set same gains and use slots 0 & 1 
     // Make sure we set the gains on both sides - left and irght - use slots 0 & 1 just sl that everything has the same gains and we don't get confused betweeen gain scheduling slots in the talons
		setSucceeded &= TalonHelper.setPIDGains(leftMaster, 0, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
		setSucceeded &= TalonHelper.setPIDGains(rightMaster, 0, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
		setSucceeded &= TalonHelper.setPIDGains(rightMaster, 1, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
		setSucceeded &= TalonHelper.setPIDGains(leftMaster, 1, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
		

		leftMaster.selectProfileSlot(0, 0);
		rightMaster.selectProfileSlot(0, 0);
     }

     @Override
     public void registerEnabledLoops(Looper in) {
          in.register(mLoop);
     }
}