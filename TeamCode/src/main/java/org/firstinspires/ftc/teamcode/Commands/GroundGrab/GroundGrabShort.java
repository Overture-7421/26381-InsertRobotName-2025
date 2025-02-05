package org.firstinspires.ftc.teamcode.Commands.GroundGrab;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;

public class GroundGrabShort extends SequentialCommandGroup {

    public GroundGrabShort(Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_GROUDGRAB_SHORT).withTimeout(1000),
                new WaitCommand(500),
                new ParallelCommandGroup( new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_GROUDGRAB_SHORT).withTimeout(1000),
                        new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_LONG).withTimeout(1000)
                )
        );

    }

}
