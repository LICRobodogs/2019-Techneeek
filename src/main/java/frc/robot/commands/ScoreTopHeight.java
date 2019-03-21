package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.Intake.IntakeState;


public class ScoreTopHeight extends Command {    
    public ScoreTopHeight() {
        requires(Robot.arm);
        requires(Robot.elevator);
    }

    protected void initialize() {
        if((Robot.arm.getDesiredSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.FRONT) || (Robot.arm.getDesiredSide() == ArmSide.BACK && Robot.arm.getSide() == ArmSide.BACK)){
            new GoToTopHeight().start();
        }else if((Robot.arm.getDesiredSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.BACK) || (Robot.arm.getDesiredSide() == ArmSide.BACK && Robot.arm.getSide() == ArmSide.FRONT)){
            new SwitchSideAndGoToTopHeight().start();
        }
    }

    //TODO: See if this needs ArmToggleFront command
    private static class SwitchSideAndGoToTopHeight extends ConditionalCommand {
        public SwitchSideAndGoToTopHeight() {
            super(new PreventElevatorCollision((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontHatch(3) : new ScoreBackHatch(3)), new PreventElevatorCollision((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontCargo(3) : new ScoreBackCargo(3)));
            System.out.println("desired side to flip:"+Robot.arm.getDesiredSide());
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }

    private static class GoToTopHeight extends ConditionalCommand {
        public GoToTopHeight() {
            super((Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontHatch(3) : new ScoreBackHatch(3), (Robot.arm.getDesiredSide()==ArmSide.FRONT) ? new ScoreFrontCargo(3) : new ScoreBackCargo(3));
            requires(Robot.intake);
        }

        @Override
        public boolean condition() {
            return Robot.intake.getSuccState() == IntakeState.SUCC_IN;
        }
    }

    protected boolean isFinished() {
        // if((Robot.arm.getSide() == ArmSide.BACK && Robot.arm.getDesiredSide() == ArmSide.BACK) || (Robot.arm.getSide() == ArmSide.FRONT && Robot.arm.getDesiredSide() == ArmSide.FRONT)){
        //     System.out.println("~~~ TOP HEIGHT DONE ~~~");
        //     return true;
        // }
        return false;
    }
}
