package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveToHatch;
import frc.robot.commands.limelight.BlinkLED;
import frc.robot.commands.limelight.OffLED;
import frc.robot.commands.limelight.OnLED;

public class DriveAndScoreHatch extends CommandGroup {
	public DriveAndScoreHatch() {
        addSequential(new OnLED());
        addSequential(new DriveToHatch());
        // addSequential(new ScoreHatch());
        addSequential(new BlinkLED());
        addSequential(new WaitCommand(1.0));
        addSequential(new OffLED());
	}
}
