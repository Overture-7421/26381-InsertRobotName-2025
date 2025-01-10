package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.overture.ftc.overftclib.Contollers.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm extends SubsystemBase {

    private final DcMotorEx right_Motor;
    private final DcMotorEx left_Motor;
    private final PIDController armPID;
    private final Telemetry telemetry;

    public static final double COUNTS_PER_REV = 8192;
    private static final double OFFSET = 31;
    public static double target = -31;
    public static double ff = 0.49;
    public static double p = 0.0495;

    public Arm(HardwareMap hardwareMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        right_Motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "right_ArmMotor");
        left_Motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "left_ArmMotor");

        armPID = new PIDController(p, 0, 0.0);

        right_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double armFeedForward(double angle){
        return ((Math.cos(Math.toRadians(angle))) * ff);
    }

    public double getPosition() {
        double currentTicks = left_Motor.getCurrentPosition();
        return ((currentTicks / COUNTS_PER_REV) * 360 - OFFSET);
    }

    public void setTarget(double targetPos) {
        target = targetPos;
    }

    @Override
    public void periodic() {
        double motorOutput= armPID.calculate(getPosition(), target);
        left_Motor.setPower(motorOutput + armFeedForward(getPosition()));
        right_Motor.setPower(motorOutput + armFeedForward(getPosition()));
    }
}