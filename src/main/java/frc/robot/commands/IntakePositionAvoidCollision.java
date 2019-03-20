package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToFrontCargo;
import frc.robot.commands.arm.ArmGoToHome;
import frc.robot.commands.arm.ArmToggleFront;
import frc.robot.commands.elevator.ElevatorGoToIntake;
import frc.robot.subsystems.Arm.ArmSide;

public class IntakePositionAvoidCollision extends ConditionalCommand {
    public IntakePositionAvoidCollision() {
        super(new SwitchSideAndGoToIntake(), new GoToIntake());
        requires(Robot.arm);
    }

    @Override
    protected boolean condition() {
        return Robot.arm.getDesiredSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.BACK;
    }

    private static class SwitchSideAndGoToIntake extends CommandGroup {
        public SwitchSideAndGoToIntake() {
            addSequential(new PreventElevatorCollision());
            addSequential(new ArmGoToFrontCargo());
            addSequential(new WaitCommand(0.25));
            addSequential(new ElevatorGoToIntake());
            addSequential(new ArmGoToHome());
            addSequential(new ArmToggleFront());
        }
    }

    private static class GoToIntake extends CommandGroup {
        public GoToIntake() {
            addSequential(new ElevatorGoToIntake());
            addSequential(new ArmGoToHome());
        }
    }
}
