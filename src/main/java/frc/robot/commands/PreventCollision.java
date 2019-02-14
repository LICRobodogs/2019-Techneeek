package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PreventCollision extends CommandGroup {
        public PreventCollision(int desiredArmPosition) {
                addSequential(new PreventElevatorCollision(desiredArmPosition));
                addSequential(new PreventArmCollision(desiredArmPosition));
        }
}
