package org.firstinspires.ftc.teamcode.Commands.Baskets;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;

public class LowBasket extends SequentialCommandGroup {

    public LowBasket (Arm arm, Elevator elevator, Intake intake){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_LOWBASKET).withTimeout(500),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_LOWBASKET).withTimeout(500),
                new MoveIntake(intake,-1.0),
                new WaitCommand(1000),
                new MoveIntake(intake,0.0),
                new StowAll(arm, elevator)

        );

    }

}
