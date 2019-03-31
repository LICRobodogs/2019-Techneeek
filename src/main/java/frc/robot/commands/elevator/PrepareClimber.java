package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareClimber extends CommandGroup {
        public PrepareClimber() {
                addParallel(new EngageClimber());
                addParallel(new StartSUCC());
                addParallel(new EngageRope());
        }
}
