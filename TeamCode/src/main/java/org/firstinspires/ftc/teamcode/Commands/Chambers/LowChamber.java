package org.firstinspires.ftc.teamcode.Commands.Chambers;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Commands.WaitForButton;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;



public class LowChamber extends SequentialCommandGroup {

    public LowChamber (Arm arm, Elevator elevator, Wrist wrist, GamepadEx operatorGamepad){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_LOWCHAMBER).withTimeout(500),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_LOWCHAMBER),
                new WaitForButton(operatorGamepad, GamepadKeys.Button.X),
                new MoveWrist(wrist, 0.4),
                new WaitForButton(operatorGamepad, GamepadKeys.Button.X),
                new MoveArm(arm,  14.0 ).withTimeout(500)
        );
    }
}