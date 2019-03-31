package frc.robot.commands.score;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.ArmGoToFrontHatch;
import frc.robot.commands.elevator.ElevatorGoToHatch;

public class ScoreFrontHatch extends CommandGroup {
        public ScoreFrontHatch(int level) {
                addSequential(new ArmGoToFrontHatch());
                addSequential(new ElevatorGoToHatch(level));
        }
}
