package frc.robot.commands;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.Intake.IntakeState;

public class ScoreBottomHeight extends ConditionalCommand {
    public ScoreBottomHeight() {
        super(new SwitchSideAndGoToBottomHeight(Robot.arm.getDesiredSide()), new GoToBottomHeight(Robot.arm.getDesiredSide()));
        requires(Robot.arm);
    }

    @Override
    protected boolean condition() {
        return Robot.arm.getSide() != Robot.arm.getDesiredSide();
    }

    private static class SwitchSideAndGoToBottomHeight extends ConditionalCommand {
        public SwitchSideAndGoToBottomHeight(ArmSide side) {
            super(new PreventElevatorCollision((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontHatch(1) : new ScoreBackHatch(1)), new PreventElevatorCollision((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontCargo(1) : new ScoreBackCargo(1)));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }

    private static class GoToBottomHeight extends ConditionalCommand {
        public GoToBottomHeight(ArmSide side) {
            super((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontHatch(1) : new ScoreBackHatch(1), (Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontCargo(1) : new ScoreBackCargo(1));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }
}
