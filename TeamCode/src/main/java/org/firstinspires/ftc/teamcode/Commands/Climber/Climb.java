package org.firstinspires.ftc.teamcode.Commands.Climber;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Commands.WaitForButton;

public class Climb extends SequentialCommandGroup {

    public Climb (Arm arm, Elevator elevator, Wrist wrist, GamepadEx operatorGamepad){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_CLIMB).withTimeout(700),
                new ParallelCommandGroup(
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_CLIMB).withTimeout(300),
                new MoveWrist(wrist, 0.7)),

                new WaitForButton(operatorGamepad, GamepadKeys.Button.Y),
                new MoveWrist(wrist, 0.1)

        );
    }

}
