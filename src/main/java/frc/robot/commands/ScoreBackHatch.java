package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.ArmGoToBackHatch;
import frc.robot.commands.elevator.ElevatorGoToHatch;

public class ScoreBackHatch extends CommandGroup {
        public ScoreBackHatch(int level) {
                addSequential(new ElevatorGoToHatch(level));
                addSequential(new ArmGoToBackHatch());
        }
}
