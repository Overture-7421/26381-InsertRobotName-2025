package org.firstinspires.ftc.teamcode.Commands;
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

public class GroundGrab extends SequentialCommandGroup {

    public  GroundGrab(Arm arm, Elevator elevator, Wrist wrist){
        addCommands(

                new MoveArm(arm, Constants.Arm.ARM_GROUNDGRAB).withTimeout(1000),
                new WaitCommand(500),
                new ParallelCommandGroup( new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_GROUNDGRAB).withTimeout(1000),
                new MoveWrist(wrist, Constants.Wrist.WRIST_GROUDGRAB).withTimeout(1000)
                )
        );

    }

}