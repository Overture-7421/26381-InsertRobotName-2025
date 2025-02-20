package org.firstinspires.ftc.teamcode.Commands.Baskets;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.ChassisPaths;
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;


public class AutoHighBasket extends SequentialCommandGroup {

    public AutoHighBasket(Chassis chassis, Arm arm, Elevator elevator, Wrist wrist){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_HIGHBASKET).withTimeout(500),
                new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_HIGHBASKET).withTimeout(2000),
                new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_MEDIUM).withTimeout(500),
                new ChassisPaths(chassis, 0, -0.4)
        );
    }
}