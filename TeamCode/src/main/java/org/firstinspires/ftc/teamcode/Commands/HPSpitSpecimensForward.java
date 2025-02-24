package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;


public class HPSpitSpecimensForward extends SequentialCommandGroup {
    public HPSpitSpecimensForward(Arm arm, Intake intake, Elevator elevator, Wrist wrist){
        addCommands(
                new MoveArm(arm, 10).withTimeout(200),
                new ElevatorPositions(elevator, 10).withTimeout(200),
                new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_MEDIUM),
                new MoveIntake(intake, Constants.Intake.INTAKE_OPEN)
        );
    }


}
