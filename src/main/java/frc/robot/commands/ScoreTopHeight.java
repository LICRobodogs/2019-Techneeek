package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.Intake.IntakeState;

public class ScoreTopHeight extends CommandGroup {
        public ScoreTopHeight() {
            if(Robot.arm.getPrevSide() == ArmSide.NEITHER || (Robot.arm.getPrevSide() == ArmSide.BACK && Robot.arm.getSide() == ArmSide.FRONT)) {
                if(Robot.intake.getSuccState() == IntakeState.SUCC_IN){
                    addSequential(new PreventArmCollision(Robot.arm.getFrontHatchPosition()));
                    addSequential(new ScoreFrontHatch(3));
                }else{
                    addSequential(new PreventArmCollision(Robot.arm.getFrontCargoPosition()));
                    addSequential(new ScoreFrontCargo(3));                    
                }
            }else if(Robot.arm.getPrevSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.FRONT){
                if(Robot.intake.getSuccState() == IntakeState.SUCC_IN){
                    addSequential(new ScoreFrontHatch(3));
                }else{
                    addSequential(new ScoreFrontCargo(3));                    
                }
            }else if(Robot.arm.getPrevSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.BACK){
                if(Robot.intake.getSuccState() == IntakeState.SUCC_IN){
                    addSequential(new PreventArmCollision(Robot.arm.getBackHatchPosition()));
                    addSequential(new ScoreBackHatch(3));
                }else{
                    addSequential(new PreventArmCollision(Robot.arm.getBackCargoPosition()));
                    addSequential(new ScoreBackCargo(3));                    
                }
            }else if(Robot.arm.getPrevSide() == ArmSide.BACK && Robot.arm.getSide() == ArmSide.BACK){
                if(Robot.intake.getSuccState() == IntakeState.SUCC_IN){
                    addSequential(new ScoreBackHatch(3));
                }else{
                    addSequential(new ScoreBackCargo(3));                    
                }
            }
        }
}
