package org.firstinspires.ftc.teamcode.Commands.Chambers;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.WaitForButton;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class AutoHighChamber extends SequentialCommandGroup {
    public AutoHighChamber(Arm arm, Elevator elevator, Wrist wrist, Intake intake){
        addCommands(
                new ParallelCommandGroup(
                        new MoveArm(arm, Constants.Arm.ARM_HIGHCHAMBER).withTimeout(1500),
                        new MoveWrist(wrist, 0.6)),
                new WaitCommand(3000),
                new ParallelCommandGroup(
                        new MoveWrist(wrist, 0.3),
                        new ElevatorPositions(elevator, 31).withTimeout(2000)),
                new MoveIntake(intake, 0.4)
        );
    }
}
