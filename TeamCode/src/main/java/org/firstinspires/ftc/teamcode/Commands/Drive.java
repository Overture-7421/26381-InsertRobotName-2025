package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import com.overture.ftc.overftclib.Utils.OverJoystickHandler;


public class Drive extends CommandBase {
    private final Chassis chassis;
    private final Gamepad driverGamepad;

    private final TrapezoidProfile leftProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.6, 999)); //max Vel, max Accel
    private final TrapezoidProfile rightProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.6,999));

    private TrapezoidProfile.State leftState = new TrapezoidProfile.State(); //Goals
    private TrapezoidProfile.State rightState = new TrapezoidProfile.State();

    public Drive (Chassis chassis, Gamepad driverGamepad) {
        this.driverGamepad = driverGamepad;
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void execute(){
        double right = -driverGamepad.right_stick_x; //Input of the Joysticks
        double left = -driverGamepad.left_stick_y;

        right = OverJoystickHandler.handleJoystickInput(right); //Asign Values of Joysticks
        left = OverJoystickHandler.handleJoystickInput(left);

        rightState = rightProfile.calculate(0.5, rightState,  new TrapezoidProfile.State(right, 0.0));
        leftState = leftProfile.calculate(0.5, leftState, new TrapezoidProfile.State(left, 0.0));

        chassis.setVelocity(leftState.position, rightState.position);
    }
}