package org.firstinspires.ftc.teamcode.Subsystems;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final DcMotorEx right_elevatorMotor;
    private final DcMotorEx left_elevatorMotor;
    private ProfiledPIDController elevatorMotorPID;
    public static final double TICKS_PER_REVOLUTION = 384.5;
    public static final double ELEVATOR_WINCH_CIRCUMFERENCE = 12.0008738;    // In Meters diameter: 3.82 cm
    public static final double GEAR_REDUCTION = 13.7;

    private double motorOffset = 0.0;

    public Elevator(HardwareMap hardwareMap) {
        right_elevatorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightelevator_Motor");
        left_elevatorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftelevator_Motor");

        elevatorMotorPID = new ProfiledPIDController(0.6,0,0, new TrapezoidProfile.Constraints(130.0,100.0));


        right_elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        left_elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        right_elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        resetZero();

        elevatorMotorPID.reset(getHeight());
        elevatorMotorPID.setGoal(getHeight());

    }

    public void resetZero() {
        motorOffset = right_elevatorMotor.getCurrentPosition();
        motorOffset = left_elevatorMotor.getCurrentPosition();
    }


    public double getRightHeight() {
        double elevatorMotorsTicks = right_elevatorMotor.getCurrentPosition();
        double elevatorMotorsCurrentHeight = elevatorMotorsTicks * ((ELEVATOR_WINCH_CIRCUMFERENCE/TICKS_PER_REVOLUTION));
        return elevatorMotorsCurrentHeight;
    }

    public double getLeftHeight() {
        double elevatorMotorsTicks = left_elevatorMotor.getCurrentPosition();
        double elevatorMotorsCurrentHeight = elevatorMotorsTicks * ((ELEVATOR_WINCH_CIRCUMFERENCE/TICKS_PER_REVOLUTION));
        return elevatorMotorsCurrentHeight;
    }


    public double getHeight() {
        double rightHeight = getRightHeight();
        double leftHeight = getLeftHeight();
        return (rightHeight + leftHeight) / 2.0;
    }


    public void setGoal(double goalHeight) {
        if(elevatorMotorPID.getGoal().position != goalHeight) {
            elevatorMotorPID.reset(getHeight());
            elevatorMotorPID.setGoal(goalHeight);
        }
    }


    //Periodic actions used for positional Elevator
    @Override
    public void periodic() {
        double outputMotor = elevatorMotorPID.calculate(getHeight());
        right_elevatorMotor.setPower(outputMotor);
        left_elevatorMotor.setPower(outputMotor);

        if(getHeight() > 97){
            right_elevatorMotor.setPower(0.0);
            left_elevatorMotor.setPower(0.0);
        }

        if(getHeight() < -0.1){
            right_elevatorMotor.setPower(0.0);
            left_elevatorMotor.setPower(0.0);
        }
    }
}
