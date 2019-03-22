package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.KillArm;
import frc.robot.commands.drivetrain.KillDrive;
import frc.robot.commands.elevator.KillElevator;

public class KillAll extends CommandGroup {
        public KillAll() {
                addParallel(new KillArm());
                addParallel(new KillElevator());
                addParallel(new KillDrive());
        }
}
