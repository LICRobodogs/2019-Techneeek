package frc.util;

public interface ControlLoopable {
	public void controlLoopUpdate();

	public void setPeriodMs(long periodMs);
}