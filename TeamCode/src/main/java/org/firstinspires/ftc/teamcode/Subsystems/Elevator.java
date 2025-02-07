package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.overture.ftc.overftclib.Contollers.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Config
public class Elevator extends SubsystemBase {

//    public static double d = 0;
//    public static double p = 0;
//    private final Telemetry telemetry;

    private final DcMotorEx right_elevatorMotor;
    private final DcMotorEx left_elevatorMotor;
    private PIDController elevatorMotorPID;
    public static final double TICKS_PER_REVOLUTION = 384.5;
    public static final double ELEVATOR_WINCH_CIRCUMFERENCE = 12.0008738;    // In Meters diameter: 3.82 cm
    public static final double GEAR_REDUCTION = 13.7;
    public static double target = 0;

    private double motorOffset = 0.0;

    public Elevator(HardwareMap hardwareMap) {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
        right_elevatorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightelevator_Motor");
        left_elevatorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftelevator_Motor");

        elevatorMotorPID = new PIDController(0.25, 0, 0);


        right_elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left_elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        right_elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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
        return getRightHeight();
//        double rightHeight = getRightHeight();
//        double leftHeight = getLeftHeight();
//        return (rightHeight + leftHeight) / 2.0;
    }


    public void setGoal(double goalHeight) {
        target = goalHeight;
    }


    //Periodic actions used for positional Elevator
    @Override
    public void periodic() {
//        elevatorMotorPID.setD(d);
//        elevatorMotorPID.setP(p);
        double outputMotor = elevatorMotorPID.calculate(getHeight(), target);

        if(getHeight() > 70) {
            right_elevatorMotor.setPower(0.0);
            left_elevatorMotor.setPower(0.0);
        } else if (getHeight() < - 0.1){
            right_elevatorMotor.setPower(0.0);
            left_elevatorMotor.setPower(0.0);
        } else {
            right_elevatorMotor.setPower(outputMotor);
            left_elevatorMotor.setPower(outputMotor);
        }

//        telemetry.addData("Target", target);
//        telemetry.addData("Current Pos", getHeight());
//        telemetry.update();
    }
}
