package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class HPSpitSpecimenReverse extends SequentialCommandGroup {

    public HPSpitSpecimenReverse (Arm arm, Intake intake, Wrist wrist){
        addCommands(
                new MoveArm(arm, Constants.Arm.ARM_CLIMB).withTimeout(700),
                new MoveWrist(wrist, 0.2),
                new MoveIntake(intake, Constants.Intake.INTAKE_OPEN)

        );
    }

}


