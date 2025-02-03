package org.firstinspires.ftc.teamcode.Commands.Grab;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
    public class GrabSpecimens extends ParallelCommandGroup {
    public GrabSpecimens (Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_LOWCHAMBER),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_STOW),
                new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_MEDIUM)
        );
    }
}
