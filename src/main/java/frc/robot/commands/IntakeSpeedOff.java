package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class IntakeSpeedOff extends CommandGroup {

	public IntakeSpeedOff() {
		addSequential(new IntakeSpeed(0.0));
	}
}
