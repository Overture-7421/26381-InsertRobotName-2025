package org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Commands.ElevatorPositions;


public class GroundGrab extends SequentialCommandGroup {

    public void GroundGrab(Arm arm, Elevator elevator){
        addCommands(
                new MoveArm(arm, 1.0),
                new WaitCommand(500),
                new ElevatorPositions(elevator, 1.0)

        );
    }

}