package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.util.Timing;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.util.concurrent.TimeUnit;

public class MoveWrist extends CommandBase {
    private Wrist wrist;
    private double wristMotorPosition;
    private Timing.Timer timer;

    public MoveWrist(Wrist wrist, double ClawMotorPosition) {
        this.wristMotorPosition = ClawMotorPosition;
        wrist = wrist;
        timer = new Timing.Timer(1, TimeUnit.SECONDS);
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.wristPosition(wristMotorPosition);
        timer.start();
    }


    @Override
    public boolean isFinished() {
        return timer.done();
    }
}

