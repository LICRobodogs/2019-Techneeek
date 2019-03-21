package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.Intake.IntakeState;


public class ScoreMiddleHeight extends Command {    
    public ScoreMiddleHeight() {
        requires(Robot.arm);
        requires(Robot.elevator);
    }

    protected void initialize() {
        if((Robot.arm.getDesiredSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.FRONT) || (Robot.arm.getDesiredSide() == ArmSide.BACK && Robot.arm.getSide() == ArmSide.BACK)){
            new GoToMiddleHeight().start();
        }else if((Robot.arm.getDesiredSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.BACK) || (Robot.arm.getDesiredSide() == ArmSide.BACK && Robot.arm.getSide() == ArmSide.FRONT)){
            new SwitchSideAndGoToMiddleHeight().start();
        }
    }

    //TODO: See if this needs ArmToggleFront command
    private static class SwitchSideAndGoToMiddleHeight extends ConditionalCommand {
        public SwitchSideAndGoToMiddleHeight() {
            super(new PreventElevatorCollision((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontHatch(2) : new ScoreBackHatch(2)), new PreventElevatorCollision((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontCargo(2) : new ScoreBackCargo(2)));
            System.out.println("desired side to flip:"+Robot.arm.getDesiredSide());
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }

    private static class GoToMiddleHeight extends ConditionalCommand {
        public GoToMiddleHeight() {
            super((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontHatch(2) : new ScoreBackHatch(2), (Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontCargo(2) : new ScoreBackCargo(2));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }

    protected boolean isFinished() {
        // if((Robot.arm.getSide() == ArmSide.BACK && Robot.arm.getDesiredSide() == ArmSide.BACK) || (Robot.arm.getSide() == ArmSide.FRONT && Robot.arm.getDesiredSide() == ArmSide.FRONT)){
        //     System.out.println("~~~ MIDDLE HEIGHT DONE ~~~");
        //     return true;
        // }
        return false;
    }
}
