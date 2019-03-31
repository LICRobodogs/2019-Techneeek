package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToDrivingPosition;
import frc.robot.commands.arm.ArmGoToFrontCargo;
import frc.robot.commands.arm.ArmToggleFront;
import frc.robot.commands.elevator.ElevatorGoToDriving;
import frc.robot.commands.elevator.PreventElevatorCollision;
import frc.robot.subsystems.Arm.ArmSide;

public class DrivingPositionAvoidCollision extends Command {
    public DrivingPositionAvoidCollision() {
        requires(Robot.arm);
        requires(Robot.elevator);
    }

    protected void initialize() {
        if (Robot.arm.getSide() != ArmSide.FRONT) {
            new SwitchSideAndGoToIntake().start();
        } else {
            new GoToIntake().start();
        }
    }

    private static class SwitchSideAndGoToIntake extends CommandGroup {
        public SwitchSideAndGoToIntake() {
            addSequential(new PreventElevatorCollision());
            addSequential(new ArmGoToFrontCargo());
            addSequential(new WaitCommand(0.25));
            addSequential(new ElevatorGoToDriving());
            addSequential(new ArmGoToDrivingPosition());
            addSequential(new ArmToggleFront());
        }
    }

    private static class GoToIntake extends CommandGroup {
        public GoToIntake() {
            addParallel(new ElevatorGoToDriving());
            addParallel(new ArmGoToDrivingPosition());
        }
    }

    protected boolean isFinished() {
        // if((Robot.arm.getSide() == ArmSide.BACK && Robot.arm.getDesiredSide() ==
        // ArmSide.BACK) || (Robot.arm.getSide() == ArmSide.FRONT &&
        // Robot.arm.getDesiredSide() == ArmSide.FRONT)){
        // System.out.println("~~~ MIDDLE HEIGHT DONE ~~~");
        // return true;
        // }
        return false;
    }
}
