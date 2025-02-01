package org.firstinspires.ftc.teamcode.Commands.Intake;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class MoveIntake extends CommandBase {

    private final Intake intake;
    private double intakePosition;

    private Timing.Timer timer;

    public MoveIntake(Intake intake, double intakePosition) {
        this.intake = intake;
        this.intakePosition = intakePosition;
        timer = new Timing.Timer(1, TimeUnit.SECONDS);
        addRequirements(intake); // Declare subsystem dependency
    }

    @Override
    public void initialize() {
        intake.IntakePosition(intakePosition);
        timer.start();
    }

    @Override
    public boolean isFinished(){
        return timer.done();
    }
}
