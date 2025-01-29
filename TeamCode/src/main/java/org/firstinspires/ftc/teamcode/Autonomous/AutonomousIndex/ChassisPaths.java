package org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import com.arcrobotics.ftclib.util.Timing;
import java.util.concurrent.TimeUnit;


public class ChassisPaths extends CommandBase {
    private final Chassis chassis;
    private double angularVel;
    private double linearVel;


    public ChassisPaths(Chassis subsystem, double angularVel, double linearVel) {
        this.angularVel = angularVel;
        this.linearVel = linearVel;
        chassis = subsystem;

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.setVelocity(linearVel, angularVel);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setVelocity(0, 0);
    }
}