package org.firstinspires.ftc.teamcode.Commands.Climber;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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
                new MoveArm(arm, Constants.Arm.ARM_CLIMB).withTimeout(300),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_CLIMB).withTimeout(300),
                new MoveWrist(wrist, 0.7),
                new WaitCommand(3000),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_STOW).withTimeout(300)
        );
    }

}
