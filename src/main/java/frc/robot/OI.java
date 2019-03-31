package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controller.GamePad;
import frc.controller.GamePad.DPadButton;
import frc.robot.commands.DriveAndScoreHatch;
import frc.robot.commands.DrivingPositionAvoidCollision;
import frc.robot.commands.KillAll;
import frc.robot.commands.arm.ArmPistonPosition;
import frc.robot.commands.arm.ArmToggleFront;
import frc.robot.commands.arm.KillArm;
import frc.robot.commands.arm.ZeroArm;
import frc.robot.commands.drivetrain.KillDrive;
import frc.robot.commands.elevator.EngageClimber;
import frc.robot.commands.elevator.EngageGravity;
import frc.robot.commands.elevator.KillElevator;
import frc.robot.commands.elevator.PrepareClimber;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.intake.IntakePositionAvoidCollision;
import frc.robot.commands.intake.IntakeSpeed;
import frc.robot.commands.intake.IntakeSpeedOff;
import frc.robot.commands.intake.IntakeSuction;
import frc.robot.commands.limelight.StartDriverCam;
import frc.robot.commands.limelight.StartVisionCam;
import frc.robot.commands.misc.UseDashBoardVariable;
import frc.robot.commands.score.ScoreBottomHeight;
import frc.robot.commands.score.ScoreHatch;
import frc.robot.commands.score.ScoreMiddleHeight;
import frc.robot.commands.score.ScoreTopHeight;
import frc.robot.subsystems.Arm.ArmPistonState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.util.Constants;

public class OI {
	private static OI instance;

	private XboxController m_driverGamepad;
	private GamePad m_operatorGamepad;

	public static OI getInstance() {
		if (instance == null) {
			instance = new OI();
		}
		return instance;
	}

	private OI() {
		m_driverGamepad = new XboxController(0);
		m_operatorGamepad = GamePad.getInstance();
		// DRIVER CONTROLS
		initAriannaButtons();
		// OPERATOR CONTROLS
		initSophieButtons();
		// DASHBORAD CONTROLS
		initInternalButtons();
		// Pneumatics Diagonostics
		initDiagnosticButtons();
	}

	public XboxController getDriverGamepad() {
		return m_driverGamepad;
	}

	public GamePad getOperatorGamepad() {
		return m_operatorGamepad;
	}

	public double getMoveInput() {
		return getDriverGamepad().getTriggerAxis(Hand.kLeft) - getDriverGamepad().getTriggerAxis(Hand.kRight);
		// return 0;
	}

	public double getSteerInput() {
		if (Math.abs(getDriverGamepad().getRawAxis(0)) > 0.15) {
			return getDriverGamepad().getRawAxis(0);
		}
		return 0;
	}

	private void initAriannaButtons() {
		JoystickButton setDriveSpeedSlow = new JoystickButton(m_driverGamepad, Constants.BACK_BUTTON);
		setDriveSpeedSlow.whenPressed(new EngageGravity());

		JoystickButton setDriveSpeedNormal = new JoystickButton(m_driverGamepad, Constants.START_BUTTON);
		setDriveSpeedNormal.whenPressed(new PrepareClimber());

		JoystickButton suctionRelease = new JoystickButton(m_driverGamepad, Constants.LEFT_BUMPER_BUTTON);
		suctionRelease.whenPressed(new IntakeSuction(IntakeState.SUCC_OUT));

		JoystickButton findTarget = new JoystickButton(m_driverGamepad, Constants.PS_RB_BUTTON);
		// findTarget.whenPressed(new FindTarget());

		// JoystickButton turnToTarget = new JoystickButton(m_driverGamepad,
		// Constants.PS_X_BUTTON);
		// findTarget.whenPressed(new TurnToTarget());

		JoystickButton driveToPort = new JoystickButton(m_driverGamepad, Constants.PS_SQUARE_BUTTON);
		// findTarget.whenPressed(new DriveToPort());

		JoystickButton driveToHatch = new JoystickButton(m_driverGamepad, Constants.PS_TRIANGLE_BUTTON);
		driveToHatch.whileHeld(new DriveAndScoreHatch());
		// driveToHatch.whenReleased(new OffLED());

		// DPadButton armGearboxDogArm = new DPadButton(m_driverGamepad,
		// DPadButton.Direction.RIGHT);
		// armGearboxDogArm.whenPressed(new
		// ArmGearboxPistonPosition(ArmGearboxState.ARM_DOG));

		// DPadButton armGearboxDogClimb = new DPadButton(m_driverGamepad,
		// DPadButton.Direction.LEFT);
		// armGearboxDogClimb.whenPressed(new
		// ArmGearboxPistonPosition(ArmGearboxState.CLIMB_DOG));
	}

	private void initSophieButtons() {
		DPadButton intakeOut = new DPadButton(m_operatorGamepad, DPadButton.Direction.LEFT);
		intakeOut.whileHeld(new IntakeSpeed(-.6));
		intakeOut.whenReleased(new IntakeSpeed(-.35));

		DPadButton intakeIn = new DPadButton(m_operatorGamepad, DPadButton.Direction.RIGHT);
		intakeIn.whileHeld(new IntakeSpeed(.45));
		intakeIn.whenReleased(new IntakeSpeed(.25));

		DPadButton intakeOff = new DPadButton(m_operatorGamepad, DPadButton.Direction.DOWN);
		intakeOff.whenPressed(new IntakeSpeedOff());

		DPadButton intakeHold = new DPadButton(m_operatorGamepad, DPadButton.Direction.UP);
		intakeHold.whenPressed(new DrivingPositionAvoidCollision());

		JoystickButton suctionGrab = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.LEFT_BUMPER_BUTTON);
		suctionGrab.whenPressed(new IntakeSuction(IntakeState.SUCC_IN));

		JoystickButton scoreHatch = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.RIGHT_BUMPER_BUTTON);
		scoreHatch.whenPressed(new ScoreHatch());

		JoystickButton intakePosition = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.X_BUTTON);
		intakePosition.whenPressed(new IntakePositionAvoidCollision());

		JoystickButton toggleFrontSide = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.BACK_BUTTON);
		toggleFrontSide.whenPressed(new ArmToggleFront());

		// JoystickButton toggleBackSide = new
		// JoystickButton(m_operatorGamepad.getJoyStick(), Constants.START_BUTTON);
		// toggleBackSide.whenPressed(new ArmToggleBack());

		JoystickButton level1 = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.A_BUTTON);
		level1.whenPressed(new ScoreBottomHeight());

		JoystickButton level2 = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.B_BUTTON);
		level2.whenPressed(new ScoreMiddleHeight());

		JoystickButton level3 = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.Y_BUTTON);
		level3.whenPressed(new ScoreTopHeight());
	}

	private void initInternalButtons() {		
		Button getData = new InternalButton();
		getData.whenPressed(new UseDashBoardVariable());
		SmartDashboard.putData("GET DA DATAAAA", getData);

		Button zeroArm = new InternalButton();
		zeroArm.whenPressed(new ZeroArm());
		SmartDashboard.putData("ZERO Arm", zeroArm);

		Button armDriving = new InternalButton();
		armDriving.whenPressed(new DrivingPositionAvoidCollision());
		SmartDashboard.putData("Arm Driving", armDriving);

		Button killArm = new InternalButton();
		killArm.whenPressed(new KillArm());
		SmartDashboard.putData("KILL ARM", killArm);

		Button killElevator = new InternalButton();
		killElevator.whenPressed(new KillElevator());
		SmartDashboard.putData("KILL ELEVATOR", killElevator);

		Button killDrive = new InternalButton();
		killDrive.whenPressed(new KillDrive());
		SmartDashboard.putData("KILL DRIVE", killDrive);

		Button killAll = new InternalButton();
		killAll.whenPressed(new KillAll());
		SmartDashboard.putData("KILL ALL", killAll);

		Button zeroElevator = new InternalButton();
		zeroElevator.whenPressed(new ZeroElevator());
		SmartDashboard.putData("Zero Elevator", zeroElevator);

		Button driverCam = new InternalButton();
		driverCam.whenPressed(new StartDriverCam());
		SmartDashboard.putData("Driver Cam", driverCam);

		Button visionCam = new InternalButton();
		visionCam.whenPressed(new StartVisionCam());
		SmartDashboard.putData("Driver Cam", visionCam);
	}

	private void initDiagnosticButtons() {
		Button testSuctionGrab = new InternalButton();
		testSuctionGrab.whenPressed(new IntakeSuction(IntakeState.SUCC_IN));
		SmartDashboard.putData("Suction Grab", testSuctionGrab);

		Button testSuctionRelease = new InternalButton();
		testSuctionRelease.whenPressed(new IntakeSuction(IntakeState.SUCC_OUT));
		SmartDashboard.putData("Suction Release", testSuctionRelease);

		Button testArmShoot = new InternalButton();
		testArmShoot.whenPressed(new ArmPistonPosition(ArmPistonState.SHOOT));
		// SmartDashboard.putData("Arm Shoot", testArmShoot);

		Button testArmReload = new InternalButton();
		testArmReload.whenPressed(new ArmPistonPosition(ArmPistonState.RELOAD));
		SmartDashboard.putData("Arm Reload", testArmReload);

		Button testArmBrake = new InternalButton();
		testArmBrake.whenPressed(new ArmPistonPosition(ArmPistonState.BRAKE));
		SmartDashboard.putData("Arm Brake", testArmBrake);

		Button testArmRelease = new InternalButton();
		testArmRelease.whenPressed(new ArmPistonPosition(ArmPistonState.RELEASE));
		SmartDashboard.putData("Arm Release", testArmRelease);

		Button testPiston = new InternalButton();
		testPiston.whenPressed(new EngageClimber());
		SmartDashboard.putData("Testing Piston", testPiston);

		// Button testArmGearboxArmDog = new InternalButton();
		// testArmGearboxArmDog.whenPressed(new
		// ArmGearboxPistonPosition(ArmGearboxState.ARM_DOG));
		// SmartDashboard.putData("Gearbox ARM Dog", testArmGearboxArmDog);

		// Button testArmGearboxClimbDog = new InternalButton();
		// testArmGearboxClimbDog.whenPressed(new
		// ArmGearboxPistonPosition(ArmGearboxState.CLIMB_DOG));
		// SmartDashboard.putData("Gearbox CLIMB Dog", testArmGearboxClimbDog);
	}
}
