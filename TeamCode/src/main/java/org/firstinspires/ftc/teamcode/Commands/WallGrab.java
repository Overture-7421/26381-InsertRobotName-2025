package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class WallGrab extends SequentialCommandGroup {

    public WallGrab (Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_HIGHBASKET).withTimeout(500),
                new ElevatorPositions(elevator, 4.0).withTimeout(400)

        );
    }
}
