package frc.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.util.Constants;

//TODO Code support for Start/Back/XBox(?) buttons.
/**
 * This is a wrapper for the WPILib Joystick class that represents an XBox
 * controller.
 * 
 * @author frc1675
 */
public class GamePad {
	private static GamePad instance = null;

	public static GamePad getInstance() {
		if (instance == null)
			instance = new GamePad(Constants.OPERATOR_GAMEPAD_USB_ID);
		return instance;
	}

	private Joystick stick;

	private GamePad(int portNumber) {
		stick = new Joystick(portNumber);
	}

	public Joystick getJoyStick() {
		return stick;
	}

	private boolean inDeadZone(double input) {
		boolean inDeadZone;
		if (Math.abs(input) < Constants.DEADZONE) {
			inDeadZone = true;
		} else {
			inDeadZone = false;
		}
		return inDeadZone;
	}

	private double getAxisWithDeadZoneCheck(double input) {
		if (inDeadZone(input)) {
			input = 0.0;
		}
		return input;
	}

	public boolean getAButton() {
		return stick.getRawButton(Constants.A_BUTTON);
	}

	public boolean getXButton() {
		return stick.getRawButton(Constants.X_BUTTON);
	}

	public boolean getBButton() {
		return stick.getRawButton(Constants.B_BUTTON);
	}

	public boolean getYButton() {
		return stick.getRawButton(Constants.Y_BUTTON);
	}

	public boolean getBackButton() {
		return stick.getRawButton(Constants.BACK_BUTTON);
	}

	public boolean getStartButton() {
		return stick.getRawButton(Constants.START_BUTTON);
	}

	public boolean getLeftBumperButton() {
		return stick.getRawButton(Constants.LEFT_BUMPER_BUTTON);
	}

	public boolean getRightBumperButton() {
		return stick.getRawButton(Constants.RIGHT_BUMPER_BUTTON);
	}

	public boolean getLeftJoystickButton() {
		return stick.getRawButton(Constants.LEFT_JOYSTICK_BUTTON);
	}

	public boolean getRightJoystickButton() {
		return stick.getRawButton(Constants.RIGHT_JOYSTICK_BUTTON);
	}

	public double getLeftXAxis() {
		return getAxisWithDeadZoneCheck(stick.getRawAxis(Constants.LEFT_X_AXIS));
	}

	public double getLeftYAxis() {
		return getAxisWithDeadZoneCheck(stick.getRawAxis(Constants.LEFT_Y_AXIS));
	}

	public double getRightXAxis() {
		return getAxisWithDeadZoneCheck(stick.getRawAxis(Constants.RIGHT_X_AXIS));
	}

	public double getRightYAxis() {
		return getAxisWithDeadZoneCheck(stick.getRawAxis(Constants.RIGHT_Y_AXIS));
	}

	public double getLeftTriggerAxis() {
		return getAxisWithDeadZoneCheck(stick.getRawAxis(Constants.LEFT_TRIGGER_AXIS));
	}

	public double getRightTriggerAxis() {
		return getAxisWithDeadZoneCheck(stick.getRawAxis(Constants.RIGHT_TRIGGER_AXIS));
	}

	/**
	 * Returns -1 if nothing is pressed, or the angle of the button pressed 0 = up,
	 * 90 = right, etc.
	 */
	public int getDpadAngle() {
		return stick.getPOV();
	}

	public boolean getRawDPadUp() {
		return getDpadAngle() == 0;
	}

	public DPadButton getDPadUp() {
		return new DPadButton(this, DPadButton.Direction.UP);
	}

	public boolean getRawDPadDown() {
		return getDpadAngle() == 180;
	}

	public DPadButton getDPadDown() {
		return new DPadButton(this, DPadButton.Direction.DOWN);
	}

	public boolean getRawDPadLeft() {
		return getDpadAngle() == 270;
	}

	public DPadButton getDPadLeft() {
		return new DPadButton(this, DPadButton.Direction.LEFT);
	}

	public boolean getRawDPadRight() {
		return getDpadAngle() == 90;
	}

	public DPadButton getDPadRight() {
		return new DPadButton(this, DPadButton.Direction.RIGHT);
	}

	/*
	 * public boolean getDPadLeft(){ return (stick.getRawAxis(LEFT_RIGHT_DPAD_AXIS)
	 * < LEFT_DPAD_TOLERANCE); }
	 * 
	 * public boolean getDPadRight(){ return (stick.getRawAxis(LEFT_RIGHT_DPAD_AXIS)
	 * > RIGHT_DPAD_TOLERANCE); }
	 * 
	 * public boolean getDPadTop(){ return (stick.getRawAxis(TOP_BOTTOM_DPAD_AXIS) <
	 * TOP_DPAD_TOLERANCE); }
	 * 
	 * public boolean getDPadBottom(){ return
	 * (stick.getRawAxis(TOP_BOTTOM_DPAD_AXIS) > BOTTOM_DPAD_TOLERANCE); }
	 */

	public boolean getLeftTrigger() {
		return (getLeftTriggerAxis() > Constants.LEFT_TRIGGER_TOLERANCE);
	}

	public boolean getRightTrigger() {
		return (getRightTriggerAxis() > Constants.RIGHT_TRIGGER_TOLERANCE);
	}

	public boolean getRightAxisUpTrigger() {
		return (getRightYAxis() < Constants.RIGHT_AXIS_UP_TOLERANCE);
	}

	public boolean getRightAxisDownTrigger() {
		return (getRightYAxis() > Constants.RIGHT_AXIS_DOWN_TOLERANCE);
	}

	public boolean getRightAxisLeftTrigger() {
		return (getRightXAxis() > Constants.RIGHT_AXIS_LEFT_TOLERANCE);
	}

	public boolean getRightAxisRightTrigger() {
		return (getRightXAxis() > Constants.RIGHT_AXIS_RIGHT_TOLERANCE);
	}

	public boolean getLeftAxisUpTrigger() {
		return (getLeftYAxis() < Constants.LEFT_AXIS_UP_TOLERANCE);
	}

	public boolean getLeftAxisDownTrigger() {
		return (getLeftYAxis() > Constants.LEFT_AXIS_DOWN_TOLERANCE);
	}

	public boolean getLeftAxisLeftTrigger() {
		return (getLeftXAxis() > Constants.LEFT_AXIS_LEFT_TOLERANCE);
	}

	public boolean getLeftAxisRightTrigger() {
		return (getLeftXAxis() > Constants.LEFT_AXIS_RIGHT_TOLERANCE);
	}

	public static class DPadButton extends Button {
		public static enum Direction {
			UP, DOWN, LEFT, RIGHT
		}

		private GamePad gamepad;
		private Direction direction;

		public DPadButton(GamePad gamepad, Direction direction) {
			this.gamepad = gamepad;
			this.direction = direction;
		}

		@Override
		public boolean get() {
			switch (direction) {
			case UP:
				return gamepad.getRawDPadUp();
			case DOWN:
				return gamepad.getRawDPadDown();
			case LEFT:
				return gamepad.getRawDPadLeft();
			case RIGHT:
				return gamepad.getRawDPadRight();
			default: // Never reached
				return false;
			}
		}
	}
}