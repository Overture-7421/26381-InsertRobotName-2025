package org.firstinspires.ftc.teamcode.Commands.Chambers;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class LowChamber extends SequentialCommandGroup {

    public LowChamber (Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_LOWCHAMBER).withTimeout(500),
                new WaitCommand(3000),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_LOWCHAMBER),
                new WaitCommand(3000),
                new StowAll(arm, elevator,wrist)
        );
    }
}