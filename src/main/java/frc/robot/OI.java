package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controller.GamePad;
import frc.controller.GamePadTriggerButton;
import frc.controller.Ps4_Controller;
import frc.robot.commands.ArmPistonPosition;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.IntakeSpeedOff;
import frc.robot.commands.IntakeSuction;
import frc.robot.commands.ScoreHatch;
import frc.robot.subsystems.Arm.ArmPistonState;
import frc.robot.subsystems.Intake.IntakeState;

public class OI {
    private static OI instance;

	private Ps4_Controller m_driverGamepad;
	private GamePad m_operatorGamepad;

	public OI() {
		m_driverGamepad = new Ps4_Controller(RobotMap.DRIVER_GAMEPAD_USB_ID);
		m_operatorGamepad = new GamePad(RobotMap.OPERATOR_GAMEPAD_USB_ID);

		// DRIVER CONTROLS

		JoystickButton armReload = new JoystickButton(m_driverGamepad.getJoyStick(), Ps4_Controller.LEFT_BUMPER_BUTTON);
		armReload.whenPressed(new ArmPistonPosition(ArmPistonState.RELOAD));

		JoystickButton scoreHatch = new JoystickButton(m_driverGamepad.getJoyStick(), Ps4_Controller.RIGHT_BUMPER_BUTTON);
		scoreHatch.whenPressed(new ScoreHatch());

		// DPadButton armGearboxDogArm = new DPadButton(m_driverGamepad, DPadButton.Direction.RIGHT);
		// armGearboxDogArm.whenPressed(new ArmGearboxPistonPosition(ArmGearboxState.ARM_DOG));

		// DPadButton armGearboxDogClimb = new DPadButton(m_driverGamepad, DPadButton.Direction.LEFT);
		// armGearboxDogClimb.whenPressed(new ArmGearboxPistonPosition(ArmGearboxState.CLIMB_DOG));

		// OPERATOR CONTROLS
		GamePadTriggerButton intakeOut = new GamePadTriggerButton(m_operatorGamepad, GamePad.LEFT_TRIGGER_AXIS);
		intakeOut.whileHeld(new IntakeSpeed());
		intakeOut.whenReleased(new IntakeSpeedOff());

		GamePadTriggerButton intakeIn = new GamePadTriggerButton(m_operatorGamepad, GamePad.RIGHT_TRIGGER_AXIS);
		intakeIn.whileHeld(new IntakeSpeed());
		intakeIn.whenReleased(new IntakeSpeedOff());

		JoystickButton intakeHold = new JoystickButton(m_operatorGamepad.getJoyStick(), GamePad.A_BUTTON);
		intakeHold.whileHeld(new IntakeSpeed(.275));
		intakeHold.whenReleased(new IntakeSpeedOff());

		JoystickButton suctionRelease = new JoystickButton(m_operatorGamepad.getJoyStick(), GamePad.LEFT_BUMPER_BUTTON);
		suctionRelease.whenPressed(new IntakeSuction(IntakeState.SUCC_OUT));

		JoystickButton suctionGrab = new JoystickButton(m_operatorGamepad.getJoyStick(), GamePad.RIGHT_BUMPER_BUTTON);
		suctionGrab.whenPressed(new IntakeSuction(IntakeState.SUCC_IN));

		// JoystickButton armBrake = new JoystickButton(m_operatorGamepad.getJoyStick(), GamePad.A_BUTTON);
		// armBrake.whenPressed(new ArmPistonPosition(ArmPistonState.BRAKE));

		// JoystickButton armRelease = new JoystickButton(m_operatorGamepad.getJoyStick(), GamePad.X_BUTTON);
		// armRelease.whenPressed(new ArmPistonPosition(ArmPistonState.RELEASE));

		// Pneumatics Diagonostics

		Button testSuctionGrab = new InternalButton();
		testSuctionGrab.whenPressed(new IntakeSuction(IntakeState.SUCC_IN));
		SmartDashboard.putData("Suction Grab", testSuctionGrab);

		Button testSuctionRelease = new InternalButton();
		testSuctionRelease.whenPressed(new IntakeSuction(IntakeState.SUCC_OUT));
		SmartDashboard.putData("Suction Release", testSuctionRelease);

		Button testArmShoot = new InternalButton();
		testArmShoot.whenPressed(new ArmPistonPosition(ArmPistonState.SHOOT));
		SmartDashboard.putData("Arm Shoot", testArmShoot);

		Button testArmReload = new InternalButton();
		testArmReload.whenPressed(new ArmPistonPosition(ArmPistonState.RELOAD));
        SmartDashboard.putData("Arm Reload", testArmReload);
        
        Button testArmBrake = new InternalButton();
		testArmBrake.whenPressed(new ArmPistonPosition(ArmPistonState.BRAKE));
		SmartDashboard.putData("Arm Brake", testArmBrake);

		Button testArmRelease = new InternalButton();
		testArmRelease.whenPressed(new ArmPistonPosition(ArmPistonState.RELEASE));
		SmartDashboard.putData("Arm Release", testArmRelease);

		// Button testArmGearboxArmDog = new InternalButton();
		// testArmGearboxArmDog.whenPressed(new ArmGearboxPistonPosition(ArmGearboxState.ARM_DOG));
		// SmartDashboard.putData("Gearbox ARM Dog", testArmGearboxArmDog);

		// Button testArmGearboxClimbDog = new InternalButton();
		// testArmGearboxClimbDog.whenPressed(new ArmGearboxPistonPosition(ArmGearboxState.CLIMB_DOG));
		// SmartDashboard.putData("Gearbox CLIMB Dog", testArmGearboxClimbDog);

	}

	public Ps4_Controller getDriverGamepad() {
		return m_driverGamepad;
	}

	public GamePad getOperatorGamepad() {
		return m_operatorGamepad;
	}
	
	public double getMoveInput() {
		return getDriverGamepad().xSpeed();
	}
	
	public double getSteerInput() {
		return getDriverGamepad().zRotation();
	}

	public static OI getInstance() {
		if (instance == null) {
			instance = new OI();
		}
		return instance;
	}
}
