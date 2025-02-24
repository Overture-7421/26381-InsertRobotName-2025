package org.firstinspires.ftc.teamcode.Commands.Baskets;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class LowBasket extends SequentialCommandGroup {

    public LowBasket (Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_LOWBASKET).withTimeout(500),
                new WaitCommand(1000),
                new ParallelCommandGroup( new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_LONG),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_LOWBASKET).withTimeout(500)
                )
        );

    }

}
