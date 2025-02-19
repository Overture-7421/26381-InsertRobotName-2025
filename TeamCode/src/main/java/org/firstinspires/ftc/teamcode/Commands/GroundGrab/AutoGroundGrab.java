package org.firstinspires.ftc.teamcode.Commands.GroundGrab;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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

public class AutoGroundGrab extends SequentialCommandGroup{
    public AutoGroundGrab(Arm arm, Elevator elevator, Wrist wrist, Intake intake){
        addCommands(
                new MoveArm(arm, 0),

                new ParallelCommandGroup(
                        new MoveArm(arm, 6),
                        new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_AUTO_GROUDGRAB_LONG).withTimeout(100),
                        new MoveIntake(intake, Constants.Intake.INTAKE_OPEN).withTimeout(100),
                        new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_LONG).withTimeout(100)
                ),

                new MoveArm(arm, -2).withTimeout(100),
                new MoveIntake(intake, Constants.Intake.INTAKE_STOW).withTimeout(100),
                new WaitCommand(500),
                new MoveArm(arm, 15).withTimeout(100)
        );
    }
}
