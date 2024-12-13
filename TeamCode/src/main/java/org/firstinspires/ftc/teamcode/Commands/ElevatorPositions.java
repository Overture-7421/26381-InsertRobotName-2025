package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;



public class ElevatorPositions extends CommandBase {

    private final Elevator elevator;
    private final double targetHeight;

    public ElevatorPositions(Elevator elevator, double targetHeight) {
        this.elevator = elevator;
        this.targetHeight = targetHeight;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setGoal(targetHeight);
    }

    @Override
    public boolean isFinished() {
        double currentHeight = elevator.getHeight();
        return Math.abs(targetHeight - currentHeight) < 0.01;
    }
}