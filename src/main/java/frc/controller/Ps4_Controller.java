package frc.controller;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.util.Constants;


public class Ps4_Controller extends Joystick {
	private static Ps4_Controller instance = null;
	public static Ps4_Controller getInstance() {
		if (instance == null)
			instance = new Ps4_Controller(Constants.DRIVER_GAMEPAD_USB_ID);
		return instance;
	}
    

    private Ps4_Controller(int axis) {
        super(axis);
    }

    public Joystick getJoyStick() {
		return this;
	}

    public double xSpeed() {
        return getLeftTrigger() - getRightTrigger();
    }

    public double zRotation() {
        return -(getLeftAnalog()+1/2);
    }

    public double getLeftTrigger() {
        return getAxisWithDeadZoneCheck(this.getRawAxis(Constants.leftTriggerAxis));
    }
    public double getRightTrigger() {
        // System.out.print(this.getRawAxis(rightTriggerAxis));
        return getAxisWithDeadZoneCheck(this.getRawAxis(Constants.rightTriggerAxis));
    }
    public double getLeftAnalog() {
        // System.out.print(this.getRawAxis(leftAnalogAxis));
        return getAxisWithDeadZoneCheck(this.getRawAxis(Constants.leftAnalogAxis));
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

    /**
	 * Returns -1 if nothing is pressed, or the angle of the button pressed 0 = up,
	 * 90 = right, etc.
	 */
	public int getDpadAngle() {
		return this.getPOV();
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
    
    public static class DPadButton extends Button {
		public static enum Direction {
			UP, DOWN, LEFT, RIGHT
		}

		private Ps4_Controller gamepad;
		private Direction direction;

		public DPadButton(Ps4_Controller gamepad, Direction direction) {
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