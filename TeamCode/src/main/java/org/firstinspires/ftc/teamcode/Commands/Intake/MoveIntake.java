package org.firstinspires.ftc.teamcode.Commands.Intake;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class MoveIntake extends CommandBase {

    private final Intake intake;
    private final double speed;

    public MoveIntake(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake); // Declare subsystem dependency
    }

    @Override
    public void execute() {
        intake.setSpeed(speed); // Set the servo speed
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop(); // Stop the servo when the command ends
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously
    }
}
