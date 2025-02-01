package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class StowAll extends SequentialCommandGroup {

    public StowAll (Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
               new ParallelCommandGroup(
                new MoveWrist(wrist, Constants.Wrist.WRIST_STOW)
,               new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_STOW).withTimeout(500)),

                new WaitCommand(500),
                new MoveArm(arm, Constants.Arm.ARM_STOW).withTimeout(300)
        );
    }
}

