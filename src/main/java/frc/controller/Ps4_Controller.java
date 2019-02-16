package frc.controller;
import edu.wpi.first.wpilibj.Joystick;
import frc.util.Constants;
import frc.controller.ButtonDebouncer;


public class Ps4_Controller extends Joystick {
    
    static final int leftTriggerAxis = Constants.leftTriggerAxis;
    static final int rightTriggerAxis = Constants.rightTriggerAxis;
    static final int leftAnalogAxis = Constants.leftAnalogAxis;
    private static final double DEADZONE = Constants.DEADZONE;
    static final int squareIndex = Constants.squareIndex;
    static final int xIndex = Constants.xIndex;
    static final int circleIndex = Constants.circleIndex;
    static final int triangleIndex = Constants.triangleIndex;
    static final int lbIndex = Constants.lbIndex;
    static final int rbIndex = Constants.rbIndex;
    static  ButtonDebouncer X;
    static  ButtonDebouncer SQUARE;
    static ButtonDebouncer TRIANGLE;
    static ButtonDebouncer CIRCLE;
    static ButtonDebouncer LB;
    static ButtonDebouncer RB;
    
	
	public enum Buttons {X,CIRCLE,SQUARE,TRIANGLE,LB,RB}
	public static enum Directions {UP, DOWN, LEFT, RIGHT}

    public Ps4_Controller(int axis) {
        super(axis);
        X = new ButtonDebouncer(getJoyStick(), xIndex);
        SQUARE = new ButtonDebouncer(getJoyStick(), squareIndex);
        TRIANGLE = new ButtonDebouncer(getJoyStick(), triangleIndex);
        CIRCLE = new ButtonDebouncer(getJoyStick(), circleIndex);
        LB = new ButtonDebouncer(getJoyStick(), lbIndex);
        RB = new ButtonDebouncer(getJoyStick(), rbIndex);
    }

	//WHY
    public Ps4_Controller getJoyStick() {
		return this;
	}

    public double xSpeed() {
        return getLeftTrigger() - getRightTrigger();
    }

    public double zRotation() {
        return -(getLeftAnalog()+1/2);
    }

    public double getLeftTrigger() {
        return getAxisWithDeadZoneCheck(this.getRawAxis(leftTriggerAxis));
    }
    public double getRightTrigger() {
        return getAxisWithDeadZoneCheck(this.getRawAxis(rightTriggerAxis));
    }
    public double getLeftAnalog() {
        return getAxisWithDeadZoneCheck(this.getRawAxis(leftAnalogAxis));
    }
    private boolean inDeadZone(double input) {
		boolean inDeadZone;
		if (Math.abs(input) < DEADZONE) {
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

	public boolean isButtonPressed(String button) {
        if (!contains(button)) {
            throw new Error("Illegal button type passed");
        }
        switch(button) {
            case "X":
                return X.get();
            case "CIRCLE":
                return CIRCLE.get();
            case "SQUARE":
                return SQUARE.get();
            case "TRIANGLE":
                return TRIANGLE.get();
            case "LB":    
                return LB.get();
            case "RB":
                return RB.get();
            default:
                return false;
        }
	}
	public static boolean contains(String button) {
        for (Buttons b: Buttons.values()) {
            if (b.name().equals(button)) {
                return true;
            }
        }
        return false;
	}
	public Directions getDPadDirection() {
		switch(getDpadAngle()) {
			case 0:
				return Directions.UP;
			case 90:
				return Directions.RIGHT;
			case 180:
				return Directions.DOWN;
			case 270:
				return Directions.LEFT;
			default:
				throw new Error("Error with angle of Dpad");
		}
	}
	public boolean isDPadDirection(String direction) {
		return direction.toUpperCase() == getDPadDirection().name();
	}
	 /**
	 * Returns -1 if nothing is pressed, or the angle of the button pressed 0 = up,
	 * 90 = right, etc.
	 */
	public int getDpadAngle() {
		return this.getPOV();
	}
}
