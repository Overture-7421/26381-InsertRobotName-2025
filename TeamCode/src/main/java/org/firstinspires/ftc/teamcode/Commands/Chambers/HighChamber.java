package org.firstinspires.ftc.teamcode.Commands.Chambers;
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class HighChamber extends SequentialCommandGroup {

    public HighChamber (Arm arm, Elevator elevator, Wrist wrist, Intake intake){
        addCommands(

                new MoveArm(arm, Constants.Arm.ARM_HIGHCHAMBER).withTimeout(1000),
                new ParallelCommandGroup(
                        new MoveWrist(wrist, 0.75),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_HIGHCHAMBER)).withTimeout(1000),
                new WaitCommand(5000),
               // new MoveWrist(wrist, 0.85),
                new MoveArm(arm, 35.0 ).withTimeout(1500)
               //new MoveIntake(intake, Constants.Intake.INTAKE_OPEN)
                //new StowAll(arm, elevator, wrist)
        );
    }
}
