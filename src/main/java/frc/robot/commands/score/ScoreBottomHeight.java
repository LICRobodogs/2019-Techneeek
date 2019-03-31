package frc.robot.commands.score;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.commands.elevator.PreventElevatorCollision;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.Intake.IntakeState;

public class ScoreBottomHeight extends Command {
    public ScoreBottomHeight() {
        requires(Robot.arm);
        requires(Robot.elevator);
    }

    protected void initialize() {
        if ((Robot.arm.getDesiredSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.FRONT)
                || (Robot.arm.getDesiredSide() == ArmSide.BACK && Robot.arm.getSide() == ArmSide.BACK)) {
            new GoToBottomHeight().start();
        } else if ((Robot.arm.getDesiredSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.BACK)
                || (Robot.arm.getDesiredSide() == ArmSide.BACK && Robot.arm.getSide() == ArmSide.FRONT)) {
            new SwitchSideAndGoToBottomHeight().start();
        }
    }

    // TODO: See if this needs ArmToggleFront command
    private static class SwitchSideAndGoToBottomHeight extends ConditionalCommand {
        public SwitchSideAndGoToBottomHeight() {
            super(new PreventElevatorCollision(
                    (Robot.arm.getDesiredSide() == ArmSide.FRONT) ? new ScoreFrontHatch(1) : new ScoreBackHatch(1)),
                    new PreventElevatorCollision((Robot.arm.getDesiredSide() == ArmSide.FRONT) ? new ScoreFrontCargo(1)
                            : new ScoreBackCargo(1)));
            System.out.println("desired side to flip:" + Robot.arm.getDesiredSide());
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }

    private static class GoToBottomHeight extends ConditionalCommand {
        public GoToBottomHeight() {
            super((Robot.arm.getDesiredSide() == ArmSide.FRONT) ? new ScoreFrontHatch(1) : new ScoreBackHatch(1),
                    (Robot.arm.getDesiredSide() == ArmSide.FRONT) ? new ScoreFrontCargo(1) : new ScoreBackCargo(1));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }

    protected boolean isFinished() {
        return false;
    }
}
