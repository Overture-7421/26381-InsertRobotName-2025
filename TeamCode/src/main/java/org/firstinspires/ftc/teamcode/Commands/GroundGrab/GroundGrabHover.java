package org.firstinspires.ftc.teamcode.Commands.GroundGrab;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.WaitForButton;

public class GroundGrabHover extends SequentialCommandGroup  {
    public GroundGrabHover(Intake intake, Arm arm, Elevator elevator, Wrist wrist, GamepadEx driverGamepad, GamepadEx operatorGamepad){
        addCommands(
                new WaitForButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER),
                new MoveArm(arm, 0).withTimeout(100),

                new WaitForButton(driverGamepad, GamepadKeys.Button.Y),

                new ParallelCommandGroup(
                        new MoveArm(arm, 10),
                        new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_GROUDGRAB_LONG).withTimeout(100),
                        new MoveIntake(intake, Constants.Intake.INTAKE_OPEN).withTimeout(100),
                        new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_LONG).withTimeout(100)
                )
        );

    }

}