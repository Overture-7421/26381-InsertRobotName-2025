package org.firstinspires.ftc.teamcode.Commands.Chambers;
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;

import org.firstinspires.ftc.teamcode.Commands.WaitForButton;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class HighChamber extends SequentialCommandGroup {


    public HighChamber (Arm arm, Elevator elevator, Wrist wrist, Intake intake, GamepadEx operator){
        addCommands(
                new MoveArm(arm, 40).withTimeout(500),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_LOWBASKET),
                new MoveWrist(wrist, 0.6),
                new WaitForButton(operator, GamepadKeys.Button.Y),
                new MoveArm(arm, 20).withTimeout(200),
                new MoveIntake(intake, Constants.Intake.INTAKE_OPEN),
                new StowAll(arm, elevator, wrist)
                );
    }
}
