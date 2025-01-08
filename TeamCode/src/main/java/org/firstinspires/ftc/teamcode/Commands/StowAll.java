package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class StowAll extends SequentialCommandGroup {

    public StowAll (Arm arm, Elevator elevator){
        addCommands(
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_STOW).withTimeout(300),
                new MoveArm(arm, Constants.Arm.ARM_STOW).withTimeout(300)
        );
    }
}

