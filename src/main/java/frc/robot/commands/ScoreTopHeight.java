package frc.robot.commands;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.Intake.IntakeState;

public class ScoreTopHeight extends ConditionalCommand {
    public ScoreTopHeight() {
        super(new SwitchSideAndGoToTopHeight(Robot.arm.getDesiredSide()), new GoToTopHeight(Robot.arm.getDesiredSide()));
        requires(Robot.arm);
    }

    @Override
    protected boolean condition() {
        return Robot.arm.getSide() != Robot.arm.getDesiredSide();
    }

    private static class SwitchSideAndGoToTopHeight extends ConditionalCommand {
        public SwitchSideAndGoToTopHeight(ArmSide side) {
            super(new PreventElevatorCollision((side==ArmSide.FRONT) ? new ScoreFrontHatch(3) : new ScoreBackHatch(3)), new PreventElevatorCollision((side==ArmSide.FRONT) ? new ScoreFrontCargo(3) : new ScoreBackCargo(3)));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }

    private static class GoToTopHeight extends ConditionalCommand {
        public GoToTopHeight(ArmSide side) {
            super((side==ArmSide.FRONT) ? new ScoreFrontHatch(3) : new ScoreBackHatch(3), (side==ArmSide.FRONT) ? new ScoreFrontCargo(3) : new ScoreBackCargo(3));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }
}
