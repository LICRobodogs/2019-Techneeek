package frc.robot.commands;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.Intake.IntakeState;

public class ScoreMiddleHeight extends ConditionalCommand {
    public ScoreMiddleHeight() {
        super(new SwitchSideAndGoToMiddleHeight(Robot.arm.getDesiredSide()), new GoToMiddleHeight(Robot.arm.getDesiredSide()));
        requires(Robot.arm);
    }

    @Override
    protected boolean condition() {
        return Robot.arm.getSide() != Robot.arm.getDesiredSide();
    }

    private static class SwitchSideAndGoToMiddleHeight extends ConditionalCommand {
        public SwitchSideAndGoToMiddleHeight(ArmSide side) {
            super(new PreventElevatorCollision((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontHatch(2) : new ScoreBackHatch(2)), new PreventElevatorCollision((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontCargo(2) : new ScoreBackCargo(2)));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }

    private static class GoToMiddleHeight extends ConditionalCommand {
        public GoToMiddleHeight(ArmSide side) {
            super((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontHatch(2) : new ScoreBackHatch(2), (Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontCargo(2) : new ScoreBackCargo(2));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }
}
