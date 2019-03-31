package frc.robot.commands.score;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.ArmGoToBackHatch;
import frc.robot.commands.elevator.ElevatorGoToHatch;

public class ScoreBackHatch extends CommandGroup {
        public ScoreBackHatch(int level) {
                addSequential(new ArmGoToBackHatch());
                addSequential(new ElevatorGoToHatch(level));
        }
}
