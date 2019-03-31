package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToFrontCargo;
import frc.robot.commands.arm.ArmGoToHome;
import frc.robot.commands.arm.ArmPistonPosition;
import frc.robot.commands.arm.ArmToggleFront;
import frc.robot.commands.arm.ReleaseArm;
import frc.robot.commands.elevator.ElevatorGoToIntake;
import frc.robot.commands.elevator.PreventElevatorCollision;
import frc.robot.subsystems.Arm.ArmPistonState;
import frc.robot.subsystems.Arm.ArmSide;

public class IntakePositionAvoidCollision extends Command {
    public IntakePositionAvoidCollision() {
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
            addSequential(new ElevatorGoToIntake());
            addSequential(new ArmGoToHome());
            addSequential(new ReleaseArm());
            addSequential(new ArmToggleFront());
            addSequential(new WaitCommand(1));
            addSequential(new ArmPistonPosition(ArmPistonState.BRAKE));
        }
    }

    private static class GoToIntake extends CommandGroup {
        public GoToIntake() {
            addSequential(new ArmGoToHome());
            addSequential(new ElevatorGoToIntake());
            addSequential(new ReleaseArm());
            addSequential(new WaitCommand(1));
            addSequential(new ArmPistonPosition(ArmPistonState.BRAKE));
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
