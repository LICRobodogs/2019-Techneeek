package frc.robot.commands.score;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.ArmGoToFrontCargo;
import frc.robot.commands.elevator.ElevatorGoToCargo;

public class ScoreFrontCargo extends CommandGroup {
        public ScoreFrontCargo(int level) {
                addSequential(new ArmGoToFrontCargo());
                addSequential(new ElevatorGoToCargo(level));
        }
}
