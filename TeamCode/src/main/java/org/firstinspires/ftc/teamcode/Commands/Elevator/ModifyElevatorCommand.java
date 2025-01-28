package org.firstinspires.ftc.teamcode.Commands.Elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ModifyElevatorCommand extends CommandBase {

    private final Elevator elevator;
    private Gamepad gamepad;

    private static final double INCREMENT = 10.0;

    public ModifyElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        if (gamepad != null) {
            if (gamepad.dpad_right) {
                elevator.setGoal(elevator.getHeight() + INCREMENT);
            } else if (gamepad.dpad_left) {
                elevator.setGoal(elevator.getHeight() - INCREMENT);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Continuous command
    }
}