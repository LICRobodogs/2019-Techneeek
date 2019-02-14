package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.Intake.IntakeState;

public class ScoreTopHeight extends CommandGroup {
        public ScoreTopHeight() {
            // if(Robot.arm.getPrevSide() == ArmSide.NEITHER) {
            //     if(Robot.intake.getSuccState() == IntakeState.SUCC_IN){
            //         addSequential(new PreventCollision(Robot.arm.getFrontHatchPosition()));
            //         addSequential(new ScoreFrontHatch(3));
            //     }else{
            //         addSequential(new PreventCollision(Robot.arm.getFrontCargoPosition()));
            //         addSequential(new ScoreFrontCargo(3));                    
            //     }
            // }else 
            if(Robot.arm.getPrevSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.FRONT){
                if(Robot.intake.getSuccState() == IntakeState.SUCC_IN){
                    System.out.println("going to suction height" + Robot.arm.getPrevSide() + Robot.arm.getSide());
                    addSequential(new ScoreFrontHatch(3));
                }else{
                    addSequential(new ScoreFrontCargo(3));                    
                }
            }else if(Robot.arm.getPrevSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.BACK){
                if(Robot.intake.getSuccState() == IntakeState.SUCC_IN){
                    addSequential(new PreventCollision(Robot.arm.getBackHatchPosition()));
                    System.out.println("going to suction height" + Robot.arm.getPrevSide() + Robot.arm.getSide());
                    addSequential(new ScoreBackHatch(3));
                }else{
                    addSequential(new PreventCollision(Robot.arm.getBackCargoPosition()));
                    addSequential(new ScoreBackCargo(3));                    
                }
            }else if(Robot.arm.getPrevSide() == ArmSide.BACK && Robot.arm.getSide() == ArmSide.FRONT){
                if(Robot.intake.getSuccState() == IntakeState.SUCC_IN){
                    addSequential(new PreventCollision(Robot.arm.getFrontHatchPosition()));
                    System.out.println("going to suction height");
                    addSequential(new ScoreFrontHatch(3));
                }else{
                    addSequential(new PreventCollision(Robot.arm.getFrontCargoPosition()));
                    addSequential(new ScoreFrontCargo(3));                    
                }
            }else if(Robot.arm.getPrevSide() == ArmSide.BACK && Robot.arm.getSide() == ArmSide.BACK){
                if(Robot.intake.getSuccState() == IntakeState.SUCC_IN){
                    System.out.println("going to suction height");
                    addSequential(new ScoreBackHatch(3));
                }else{
                    addSequential(new ScoreBackCargo(3));                    
                }
            }
        }
}
