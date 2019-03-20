package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.drivetrain.FancyDriveToHatch;
import frc.robot.commands.limelight.BlinkLED;
import frc.robot.commands.limelight.OffLED;
import frc.robot.commands.limelight.OnLED;

public class DriveAndScoreHatch extends CommandGroup {
	public DriveAndScoreHatch() {
        addSequential(new OnLED());
        addSequential(new WaitCommand(0.5));
        addSequential(new FancyDriveToHatch());
        // addSequential(new ScoreHatch());
        addSequential(new BlinkLED());
        addSequential(new WaitCommand(0.75));
        addSequential(new OffLED());
	}
}
