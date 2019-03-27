package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class TakeSnapShot extends CommandGroup {
	public TakeSnapShot() {
        addSequential(new StartSnapShots());
        addSequential(new WaitCommand(1.0));
        addSequential(new StopSnapShots());
	}
}
