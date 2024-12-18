package org.firstinspires.ftc.teamcode.Commands;
/*
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Controllers.FRCTrapezoidProfile;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Utils.JoystickHandler;

import com.overture.ftc.overftclib.Contollers.PIDController;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;


public class MoveChassis extends CommandBase {
    private final Chassis chassis;
    private final Gamepad driverGamepad;

    private final FRCTrapezoidProfile leftProfile = new FRCTrapezoidProfile(new FRCTrapezoidProfile.Constraints(0.6, 999));
    private final FRCTrapezoidProfile rightProfile = new FRCTrapezoidProfile(new FRCTrapezoidProfile.Constraints(0.6,999));

    private FRCTrapezoidProfile.State leftGoal = new FRCTrapezoidProfile.State();
    private FRCTrapezoidProfile.State rightGoal = new FRCTrapezoidProfile.State();

    public MoveChassis(Chassis subsystem, Gamepad driverGamepad) {
        this.driverGamepad = driverGamepad;
        chassis = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        double right = -driverGamepad.right_stick_x;
        double left = -driverGamepad.left_stick_y;

        right = JoystickHandler.handleJoystickInput(right);
        left = JoystickHandler.handleJoystickInput(left);

        rightGoal = rightProfile.calculate(0.5, rightGoal, new FRCTrapezoidProfile.State(right, 0.0));
        leftGoal = leftProfile.calculate(0.5, leftGoal, new FRCTrapezoidProfile.State(left, 0.0));

        chassis.setSpeed(leftGoal.position, rightGoal.position);
    }
}
*/
