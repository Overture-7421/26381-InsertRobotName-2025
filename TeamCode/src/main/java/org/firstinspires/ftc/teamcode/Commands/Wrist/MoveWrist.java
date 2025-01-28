package org.firstinspires.ftc.teamcode.Commands.Wrist;

import com.arcrobotics.ftclib.util.Timing;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.util.concurrent.TimeUnit;

public class MoveWrist extends CommandBase {
    private Wrist wrist;
    private double wristMotorsPosition;
    private Timing.Timer timer;

    public MoveWrist(Wrist wrist, double wristMotorsPosition) {
        this.wristMotorsPosition = wristMotorsPosition;
        this.wrist = wrist;
        timer = new Timing.Timer(1, TimeUnit.SECONDS);
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.wristPosition(wristMotorsPosition);
        timer.start();
    }


    @Override
    public boolean isFinished() {
        return timer.done();
    }
}

