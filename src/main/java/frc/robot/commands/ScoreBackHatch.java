package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.ArmGoToBackHatch;
import frc.robot.commands.elevator.ElevatorGoToHatch;
import frc.robot.subsystems.Arm.ArmSide;

public class ScoreBackHatch extends CommandGroup {
        public ScoreBackHatch(int level) {
                addSequential(new ArmGoToBackHatch());
                addSequential(new ElevatorGoToHatch(level));
        }
}
