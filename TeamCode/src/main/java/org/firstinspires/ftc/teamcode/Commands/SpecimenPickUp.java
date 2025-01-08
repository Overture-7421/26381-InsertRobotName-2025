package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;

public class SpecimenPickUp extends SequentialCommandGroup {

    public  SpecimenPickUp(Arm arm, Elevator elevator){
        addCommands(

                new MoveArm(arm, 1.0),
                new WaitCommand(500),
                new ElevatorPositions(elevator, 1.0)

        );


    }

}