package org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.Subsystems.Chassis;

public class TurnToAngle extends CommandBase {
    private final Chassis chassis;
    private final ProfiledPIDController pidController;
    private final Rotation2d targetHeading;

    public TurnToAngle(Chassis chassis, Rotation2d targetHeading){
        this.chassis = chassis;
        this.targetHeading = targetHeading;

        pidController = new ProfiledPIDController(0.1, 0.0, 0, new TrapezoidProfile.Constraints(50, 30));
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        pidController.setTolerance(2.5);
        pidController.enableContinuousInput(-180.0, 180.0);

        double targetDegrees = targetHeading.getDegrees();
        double currentDegrees = chassis.getPose().getRotation().getDegrees();

        pidController.reset(currentDegrees, 0.0);
        pidController.setGoal(targetDegrees);
    }

    @Override
    public void execute() {
        double currentDegrees = chassis.getPose().getRotation().getDegrees();

        double angularVel = pidController.calculate(currentDegrees);
        chassis.setVelocity(0.0, angularVel);
    }

    @Override
    public void end(boolean interrupted){
        chassis.setVelocity(0.0, 0.0);
    }

    @Override
    public boolean isFinished(){
        double currentDegrees = chassis.getPose().getRotation().getDegrees();
        double targetDegrees = targetHeading.getDegrees();
        return Math.abs(targetDegrees - currentDegrees) < 2.5;
    }
}