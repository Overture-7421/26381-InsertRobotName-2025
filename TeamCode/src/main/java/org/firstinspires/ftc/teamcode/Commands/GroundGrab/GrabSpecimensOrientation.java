package org.firstinspires.ftc.teamcode.Commands.GroundGrab;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Commands.WaitForButton;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class GrabSpecimensOrientation extends SequentialCommandGroup {
    public GrabSpecimensOrientation (Arm arm, GamepadEx operatorGamepad, Wrist wrist, Intake intake, Elevator elevator){
        addCommands(
                new MoveArm(arm,5.0).withTimeout(100),
                new MoveWrist(wrist, 0.5),
                new MoveIntake(intake, Constants.Intake.INTAKE_OPEN),
                new WaitForButton(operatorGamepad, GamepadKeys.Button.START),
                new MoveIntake(intake, Constants.Intake.INTAKE_STOW),
                new MoveArm(arm, 15.0).withTimeout(200),
                new MoveWrist(wrist, Constants.Wrist.WRIST_STOW),
                new StowAll(arm, elevator, wrist)
        );
    }
}



