package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToHome;
import frc.robot.commands.elevator.ElevatorGoToIntake;
import frc.robot.subsystems.Arm.ArmSide;

public class IntakePositionAvoidCollision extends ConditionalCommand {
    public IntakePositionAvoidCollision() {
        super(new SwitchSideAndGoToIntake(), new GoToIntake());
        requires(Robot.arm);
    }

    @Override
    protected boolean condition() {
        return (Robot.arm.getSide() != Robot.arm.getPrevSide()) && Robot.arm.getSide() != ArmSide.SAME;
    }

    private static class SwitchSideAndGoToIntake extends CommandGroup {
        public SwitchSideAndGoToIntake() {
            addSequential(new PreventElevatorCollision());
            addSequential(new ArmGoToHome());
            addSequential(new ElevatorGoToIntake());
        }
    }

    private static class GoToIntake extends CommandGroup {
        public GoToIntake() {
            addSequential(new ArmGoToHome());
            addSequential(new ElevatorGoToIntake());
        }
    }
}
