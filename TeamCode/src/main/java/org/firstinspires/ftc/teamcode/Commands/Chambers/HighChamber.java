package org.firstinspires.ftc.teamcode.Commands.Chambers;
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class HighChamber extends SequentialCommandGroup {

    public HighChamber (Arm arm, Elevator elevator, Wrist wrist){
        addCommands(

                new MoveArm(arm, Constants.Arm.ARM_HIGHCHAMBER),
                new WaitCommand(5000),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_HIGHCHAMBER),
                new WaitCommand(5000),
                new MoveArm(arm, 24),
                new WaitCommand(5000),
                new StowAll(arm, elevator, wrist)
        );
    }
}
