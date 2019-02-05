package frc.controller;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.util.Constants;

public class GamePadTriggerButton extends Button {

	private GamePad m_controller;
	private int m_trigger;

	public GamePadTriggerButton(GamePad controller, int trigger) {
		m_controller = controller;
		m_trigger = trigger;
	}

	public boolean get() {
		if (m_trigger == Constants.RIGHT_TRIGGER) {
			return m_controller.getRightTrigger();
		} else if (m_trigger == Constants.LEFT_TRIGGER) {
			return m_controller.getLeftTrigger();
		}
		/*
		 * else if (m_trigger == RIGHT_AXIS_UP_TRIGGER) { return
		 * m_controller.getRightAxisUpTrigger(); } else if (m_trigger ==
		 * RIGHT_AXIS_DOWN_TRIGGER) { return m_controller.getRightAxisDownTrigger(); }
		 */
		else if (m_trigger == Constants.RIGHT_AXIS_RIGHT_TRIGGER) {
			return m_controller.getRightAxisRightTrigger();
		} else if (m_trigger == Constants.RIGHT_AXIS_LEFT_TRIGGER) {
			return m_controller.getRightAxisLeftTrigger();
		} else if (m_trigger == Constants.LEFT_AXIS_UP_TRIGGER) {
			return m_controller.getLeftAxisUpTrigger();
		} else if (m_trigger == Constants.LEFT_AXIS_DOWN_TRIGGER) {
			return m_controller.getLeftAxisDownTrigger();
		} else if (m_trigger == Constants.LEFT_AXIS_RIGHT_TRIGGER) {
			return m_controller.getLeftAxisRightTrigger();
		} else if (m_trigger == Constants.LEFT_AXIS_LEFT_TRIGGER) {
			return m_controller.getLeftAxisLeftTrigger();
		}
		return false;
	}

}
