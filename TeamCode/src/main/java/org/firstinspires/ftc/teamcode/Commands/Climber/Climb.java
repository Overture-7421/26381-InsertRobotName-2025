package org.firstinspires.ftc.teamcode.Commands.Climber;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class Climb extends SequentialCommandGroup {

    public Climb (Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
                new MoveWrist(wrist, Constants.Wrist.WRIST_STOW),
                new MoveArm(arm, Constants.Arm.ARM_CLIMB),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_CLIMB)
        );
    }

}
