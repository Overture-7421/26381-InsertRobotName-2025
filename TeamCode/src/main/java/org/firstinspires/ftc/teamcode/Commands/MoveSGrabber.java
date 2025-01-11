package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.util.Timing;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.SGrabber;
import java.util.concurrent.TimeUnit;

public class MoveSGrabber extends CommandBase {
    private SGrabber sGrabber;
    private double Sgrabber_Position;
    private Timing.Timer timer;

    public MoveSGrabber(SGrabber sGrabber, double Sgrabber_Position) {
        this.Sgrabber_Position = Sgrabber_Position;
        this.sGrabber = sGrabber;
        timer = new Timing.Timer(1, TimeUnit.SECONDS);
        addRequirements(sGrabber);
    }

    @Override
    public void initialize() {
        sGrabber.setSgrabber_Position(Sgrabber_Position);
        timer.start();
    }


    @Override
    public boolean isFinished() {
        return timer.done();
    }
}

