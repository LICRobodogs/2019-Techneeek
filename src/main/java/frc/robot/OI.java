package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controller.GamePad;
import frc.controller.GamePad.DPadButton;
import frc.controller.Ps4_Controller;
import frc.robot.commands.IntakePositionAvoidCollision;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.IntakeSpeedOff;
import frc.robot.commands.IntakeSuction;
import frc.robot.commands.ScoreBottomHeight;
import frc.robot.commands.ScoreHatch;
import frc.robot.commands.ScoreMiddleHeight;
import frc.robot.commands.ScoreTopHeight;
import frc.robot.commands.arm.ArmGoToBackCargo;
import frc.robot.commands.arm.ArmGoToDrivingPosition;
import frc.robot.commands.arm.ArmGoToRest;
import frc.robot.commands.arm.ArmPistonPosition;
import frc.robot.commands.arm.ArmToggleBack;
import frc.robot.commands.arm.ArmToggleFront;
import frc.robot.subsystems.Arm.ArmPistonState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.util.Constants;

public class OI {
	private static OI instance;

	private Ps4_Controller m_driverGamepad;
	private GamePad m_operatorGamepad;

	private OI() {
		m_driverGamepad = Ps4_Controller.getInstance();
		m_operatorGamepad = GamePad.getInstance();

		// DRIVER CONTROLS

		JoystickButton armReload = new JoystickButton(m_driverGamepad.getJoyStick(), Constants.LEFT_BUMPER_BUTTON);
		armReload.whenPressed(new ArmPistonPosition(ArmPistonState.RELOAD));

		// DPadButton armGearboxDogArm = new DPadButton(m_driverGamepad, DPadButton.Direction.RIGHT);
		// armGearboxDogArm.whenPressed(new ArmGearboxPistonPosition(ArmGearboxState.ARM_DOG));
		
		// DPadButton armGearboxDogClimb = new DPadButton(m_driverGamepad, DPadButton.Direction.LEFT);
		// armGearboxDogClimb.whenPressed(new ArmGearboxPistonPosition(ArmGearboxState.CLIMB_DOG));
		
		// OPERATOR CONTROLS
		DPadButton intakeOut = new DPadButton(m_operatorGamepad, DPadButton.Direction.LEFT);
		intakeOut.whileHeld(new IntakeSpeed(-.6));
		intakeOut.whenReleased(new IntakeSpeedOff());		
		
		DPadButton intakeIn = new DPadButton(m_operatorGamepad, DPadButton.Direction.RIGHT);
		intakeIn.whileHeld(new IntakeSpeed(.45));
		intakeIn.whenReleased(new IntakeSpeed(.25));		
		
		DPadButton intakeHold = new DPadButton(m_operatorGamepad,DPadButton.Direction.UP);
		intakeHold.whenPressed(new ArmGoToDrivingPosition());

		JoystickButton suctionGrab = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.LEFT_BUMPER_BUTTON);
		suctionGrab.whenPressed(new IntakeSuction(IntakeState.SUCC_IN));

		JoystickButton scoreHatch = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.RIGHT_BUMPER_BUTTON);
		scoreHatch.whenPressed(new ScoreHatch());

		JoystickButton intakePosition = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.X_BUTTON);
		intakePosition.whenPressed(new IntakePositionAvoidCollision());
		
		JoystickButton toggleFrontSide = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.BACK_BUTTON);
		toggleFrontSide.whenPressed(new ArmToggleFront());

		JoystickButton toggleBackSide = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.START_BUTTON);
		toggleBackSide.whenPressed(new ArmToggleBack());

		JoystickButton level1 = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.A_BUTTON);
		level1.whenActive(new ScoreBottomHeight());

		JoystickButton level2 = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.B_BUTTON);
		level2.whenActive(new ScoreMiddleHeight());

		JoystickButton level3 = new JoystickButton(m_operatorGamepad.getJoyStick(), Constants.Y_BUTTON);
		level3.whenActive(new ScoreTopHeight());

		Button armRest = new InternalButton();
		armRest.whenPressed(new ArmGoToRest());
		SmartDashboard.putData("Arm Rest", armRest);

		Button armBackCargo = new InternalButton();
		armBackCargo.whenPressed(new ArmGoToBackCargo());
		SmartDashboard.putData("Arm Back Cargo", armBackCargo);
		
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
