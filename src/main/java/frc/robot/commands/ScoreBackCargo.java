package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.ArmGoToBackCargo;
import frc.robot.commands.elevator.ElevatorGoToCargo;

public class ScoreBackCargo extends CommandGroup {
        public ScoreBackCargo(int level) {
                addSequential(new ArmGoToBackCargo());
                addSequential(new ElevatorGoToCargo(level));
        }
}
