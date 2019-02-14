package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.ArmGoToFrontHatch;
import frc.robot.commands.elevator.ElevatorGoToHatch;

public class ScoreFrontHatch extends CommandGroup {
        public ScoreFrontHatch(int level) {
                addSequential(new ElevatorGoToHatch(level));
                addSequential(new ArmGoToFrontHatch());
        }
}
