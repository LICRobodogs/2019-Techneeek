package frc.robot.commands;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.Intake.IntakeState;

public class ScoreMiddleHeight extends ConditionalCommand {
    public ScoreMiddleHeight() {
        super(new SwitchSideAndGoToMiddleHeight(Robot.arm.getSide(),Robot.arm.getPrevSide()), new GoToMiddleHeight(Robot.arm.getSide(),Robot.arm.getPrevSide()));
        requires(Robot.arm);
    }

    @Override
    protected boolean condition() {
        return (Robot.arm.getSide() != Robot.arm.getPrevSide()) && Robot.arm.getSide() != ArmSide.SAME;
    }

    private static class SwitchSideAndGoToMiddleHeight extends ConditionalCommand {
        public SwitchSideAndGoToMiddleHeight(ArmSide side, ArmSide prevSide) {
            super(new PreventElevatorCollision((side==ArmSide.FRONT || (side==ArmSide.SAME && prevSide==ArmSide.FRONT)) ? new ScoreFrontHatch(2) : new ScoreBackHatch(2)), new PreventElevatorCollision((side==ArmSide.FRONT || (side==ArmSide.SAME && prevSide==ArmSide.FRONT)) ? new ScoreFrontCargo(2) : new ScoreBackCargo(2)));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }

    private static class GoToMiddleHeight extends ConditionalCommand {
        public GoToMiddleHeight(ArmSide side, ArmSide prevSide) {
            super((side==ArmSide.FRONT || (side==ArmSide.SAME && prevSide==ArmSide.FRONT)) ? new ScoreFrontHatch(2) : new ScoreBackHatch(2), (side==ArmSide.FRONT || (side==ArmSide.SAME && prevSide==ArmSide.FRONT)) ? new ScoreFrontCargo(2) : new ScoreBackCargo(2));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }
}
