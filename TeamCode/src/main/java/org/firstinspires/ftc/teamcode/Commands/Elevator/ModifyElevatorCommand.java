package org.firstinspires.ftc.teamcode.Commands.Elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ModifyElevatorCommand extends CommandBase {
    private final Elevator elevator;
    private final Gamepad gamepad;

    private static final double INCREMENT = 5.0;

    public ModifyElevatorCommand(Elevator elevator, Gamepad gamepad) {
        this.elevator = elevator;
        this.gamepad = gamepad;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (gamepad.dpad_right) {
            elevator.setGoal(elevator.getHeight() + INCREMENT);
        } else if (gamepad.dpad_left) {
            elevator.setGoal(elevator.getHeight() - INCREMENT);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Continuous command
    }
}
