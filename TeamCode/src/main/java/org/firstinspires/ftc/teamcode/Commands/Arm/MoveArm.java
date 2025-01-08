package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class MoveArm extends CommandBase {

    private final Arm arm;

    private final double targetPosition;

    public MoveArm(Arm subsystem, double targetPosition) {
        this.arm = subsystem;
        this.targetPosition = targetPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTarget(targetPosition);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = arm.getPosition();
        return Math.abs(targetPosition - currentPosition) < 0.5;
    }
}