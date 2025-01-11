package org.firstinspires.ftc.teamcode.Commands.Baskets;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.MoveWrist;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;



public class HighBasket extends SequentialCommandGroup {

    public HighBasket (Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_HIGHBASKET).withTimeout(500),

                new ParallelCommandGroup( new MoveWrist(wrist, 0.3),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_HIGHBASKET).withTimeout(500)
                )

        );

    }
}