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

    public class GrabSpecimens extends SequentialCommandGroup {
    public GrabSpecimens (Arm arm, GamepadEx operatorGamepad, Wrist wrist, Intake intake, Elevator elevator){
        addCommands(
                new MoveIntake(intake, Constants.Intake.INTAKE_STOW),
                new MoveWrist(wrist, 0.05),
                new WaitForButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER),
                new MoveArm(arm, 0),
                new StowAll(arm, elevator, wrist)

        );
    }
}
