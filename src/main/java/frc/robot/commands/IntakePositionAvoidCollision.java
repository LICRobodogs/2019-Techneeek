package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToHome;
import frc.robot.commands.elevator.ElevatorGoToIntake;

public class IntakePositionAvoidCollision extends CommandGroup {
        public IntakePositionAvoidCollision() {
            addSequential(new PreventCollision(Robot.arm.getHomePosition()));
            addSequential(new ArmGoToHome());
            addSequential(new ElevatorGoToIntake());
            
        }
}
