package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.JoystickDrive;
import frc.util.Constants;
import frc.util.Util;
import frc.util.TrajectoryFollowingMotion.Path;
import frc.util.drivers.Controllers;
import frc.util.drivers.DriveControlState;
import frc.util.drivers.DriveMotorValues;
import frc.util.drivers.DunkTalonSRX;
import frc.util.drivers.DunkVictorSPX;
import frc.util.loops.Loop;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.EncoderFollower;
import frc.robot.PathFiles.paths;

public class DriveBaseSubsystem extends PIDSubsystem {
     private static DriveBaseSubsystem instance = null;
     private DunkTalonSRX leftMaster, leftSlave, rightMaster;
     private DunkVictorSPX rightSlave;
     // should probably put the gyro here... why isn't it here
     private DriveControlState controlMode;
     private static ReentrantLock _subsystemMutex = new ReentrantLock();
     private boolean previousBrakeModeVal;

     public DifferentialDrive m_drive;

     public PIDController turnController;
     private boolean pathFindingFinished = false;
     private final double kp = 0.0;
     private final double ki = 0.0;
     private final double kd = 0.0;
     private final double kf = 0.0;
     private EncoderFollower m_left_follower, m_right_follower;
     private Controllers robotControllers;

     public static DriveBaseSubsystem getInstance() {
          if (instance == null)
               instance = new DriveBaseSubsystem();
          return instance;
     }

     private DriveBaseSubsystem() {
          super("DriveBaseSubsytem", 0.8, 0.0, 0.0);
          robotControllers = Controllers.getInstance();
          robotControllers.passPID(getPIDController());
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

          // controlMode = DriveControlState.PATH_FOLLOWING; //because we start match on
          // auton
          setAbsoluteTolerance(0.05);
          getPIDController().setContinuous(false);
          LiveWindow.addActuator("DriveBaseSubsytem", "Base", getPIDController());
     }

     public boolean isFollowingPath() {
          return (m_left_follower.isFinished() || m_right_follower.isFinished());
     }

     public void followPath() {
          double left_speed = m_left_follower.calculate(getLeftPositionRaw());
          double right_speed = m_right_follower.calculate(getRightPositionRaw());
          double heading = robotControllers.getGyro().getAngle();
          double desired_heading = -Pathfinder.r2d(m_left_follower.getHeading());
          double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);

          SmartDashboard.putNumber("Heading", heading);
          SmartDashboard.putNumber("Heading Desired", desired_heading);
          SmartDashboard.putNumber("Heading Difference", heading_difference);

          double kP = 0.005; // propotional turning constant
          double turningValue = (desired_heading - heading) * kP;
          // Invert the direction of the turn if we are going backwards //why? wouldn't
          // you negate daSpeed?

          double turn = 0.8 * (-1.0 / 80.0) * heading_difference;
          // driveBaseSubsystem.setSpeed(left_speed + turn, right_speed - turn);
          double daSpeed = (left_speed + right_speed) / 2;
          // daSpeed = left_speed;
          // turningValue = Math.copySign(turningValue, Robot.getPsController().xSpeed());
          turningValue = Math.copySign(turningValue, daSpeed);
          drive(-left_speed, turningValue);
     }
     public void initializePath(paths trajectories) {
          double kP = .4;
          double kd = kP / 100;
          pathFindingFinished = false;          
          m_left_follower = new EncoderFollower(trajectories.left);
          m_right_follower = new EncoderFollower(trajectories.right);

          m_left_follower.configureEncoder((int) getLeftDistanceInches(), Constants.DRIVE_TICKS_PER_ROTATION,
                    Constants.kDriveWheelDiameterInches);
          m_right_follower.configureEncoder((int) getRightDistanceInches(), Constants.DRIVE_TICKS_PER_ROTATION,
                    Constants.kDriveWheelDiameterInches);

          m_left_follower.configurePIDVA(kP, 0.0, kd, Constants.lKv, Constants.lKa);
          m_right_follower.configurePIDVA(kP, 0.0, kd, Constants.rKv, Constants.rKa);
     }

     private final Loop mLoop = new Loop() {
          public void onFirstStart(double timestamp) {
               synchronized (DriveBaseSubsystem.this) {
                    subsystemHome();
               }
          }

          @Override
          public void onStart(double timestamp) {
               synchronized (DriveBaseSubsystem.this) {
                    setDriveOpenLoop(DriveMotorValues.NEUTRAL);
                    setBrakeMode(false);
                    setDriveVelocity(new DriveMotorValues(0, 0));
               }
          }

          @Override
          public void onLoop(double timestamp, boolean isAuto) {
               synchronized (DriveBaseSubsystem.this) {
                    switch (controlMode) {
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
          public void onStop(double timestamp) {
               synchronized (DriveBaseSubsystem.this) {
                    setDriveOpenLoop(DriveMotorValues.NEUTRAL);
               }
          }
     };

     public void drive(double move, double steer) {
          m_drive.curvatureDrive(move, steer, true);
     }

     public void steer(double steer) {
          drive(Controllers.getInstance().getPS_Controller().xSpeed(), steer);
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
               setSucceeded &= leftMaster.getSensorCollection().setQuadraturePosition(0,
                         Constants.kMSTimeoutsFast) == ErrorCode.OK;
               setSucceeded &= rightMaster.getSensorCollection().setQuadraturePosition(0,
                         Constants.kMSTimeoutsFast) == ErrorCode.OK;
          } while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
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

               } catch (Exception e) {
                    System.err.println("Error with setting drice control mode");
               } finally {
                    _subsystemMutex.unlock();
               }
          }
     }

     public void setSpeed(double speed1, double speed2) {
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

     public void setBrakeMode(boolean brakeMode) {
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
          return rotationsToInches(leftMaster.getSelectedSensorPosition(0) / Constants.kSensorUnitsPerRotation);
     }

     public double inchesToSensorPosition(double inches) {
          return (inches * Constants.kSensorUnitsPerRotation) / (Constants.kDriveWheelDiameterInches * Math.PI);
     }

     public double getRightDistanceInches() {
          return rotationsToInches(rightMaster.getSelectedSensorPosition(0) / Constants.kSensorUnitsPerRotation);
     }

     public int getRightPositionRaw() {
          return rightMaster.getSelectedSensorPosition(0);
     }

     public int getLeftPositionRaw() {
          return leftMaster.getSelectedSensorPosition(0);
     }

     public double getLeftVelocityInchesPerSec() {
          return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(leftMaster.getSelectedSensorVelocity(0)));
     }

     public double getRightVelocityInchesPerSec() {
          return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(rightMaster.getSelectedSensorVelocity(0)));
     }

     public double getGyroAngle() {
          return robotControllers.getGyro().getAngle();
     }

     public DunkTalonSRX getLeftMaster() {
          return leftMaster;
     }

     public DunkTalonSRX getRightMaster() {
          return rightMaster;
     }

     public void dashUpdate() {
          SmartDashboard.putNumber("\nLeft Velocity (in/sec)", getLeftVelocityInchesPerSec());
          SmartDashboard.putNumber("\nRight Velocity (in/sec)", getRightVelocityInchesPerSec());
          SmartDashboard.putNumber("\nGyro Angle in Degrees", getGyroAngle());
          SmartDashboard.putNumber("\nGyro Yaw Angle in Degrees", getGyroAngle());
          SmartDashboard.putNumber("\nLeft Distance (in)", getLeftDistanceInches());
          SmartDashboard.putNumber("\nRight Distance (in)", getRightDistanceInches());
     }

     /**
      * Configures the drivebase to drive a path. Used for autonomous driving
      */
     public synchronized void setWantDrivePath(Path path, boolean reversed) {
          // ef dat code but i want dis method
          // todo
     }

     @Override
     protected void initDefaultCommand() {
          setDefaultCommand(new JoystickDrive());
     }

     // @Override
     // public void pidWrite(double output) {
     // // setControlMode(DriveControlState.OPEN_LOOP);
     // leftMaster.set(ControlMode.PercentOutput, output);
     // rightMaster.set(ControlMode.PercentOutput, output);
     // }

     // @Override
     // protected double returnPIDInput() {
     //      return leftMaster.pidGet();
     // }

     // @Override
     // protected void usePIDOutput(double output) {
     //      drive(output, 0);
     // }
     protected void usePIDOutput(double output) {
		leftMaster.pidWrite(output);
		rightMaster.pidWrite(output);
	}
	protected double returnPIDInput() {
		return getLeftDistanceInches(); // returns the sensor value that is providing the feedback for the system
	}
}