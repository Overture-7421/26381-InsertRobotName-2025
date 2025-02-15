package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Config
public class Arm extends SubsystemBase {

    private final DcMotorEx right_Motor;
    private final DcMotorEx left_Motor;
    private final PIDController armPID;
    public final DigitalChannel pushButton;
    public double ActiveButtonReset = 0;

//    private final Telemetry telemetry;
    private final DigitalChannel limitSwitch;

    public static final double COUNTS_PER_REV = 8192;
    private static final double OFFSET = 31;
    public static double target = -31;

    public static double ff = 0.06;
    public static double pUpper = 0.05;
    public static double pLower = 0.01;

    public static double dUpper = 0.00117;
    public static double dLower = 0.0005;


    private double currentOffset = 0;
    private final int expectedZeroPosition = 0;
    private boolean limitSwitchState = false;

    public Arm(HardwareMap hardwareMap) {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
        right_Motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "right_ArmMotor");
        left_Motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "left_ArmMotor");

        armPID = new PIDController(0, 0, 0);
        armPID.setTolerance(1);

        right_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "armLimitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        pushButton = hardwareMap.get(DigitalChannel.class, "arm_touch");
        pushButton.setMode(DigitalChannel.Mode.INPUT);
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


    public void decreaseTarget(){
        target = target - 1;
    }

    public void increaseTarget(){
        target = target + 1;
    }

    public boolean isLimitReached() {
        return limitSwitch.getState();
    }

    private boolean limitSwitchPreviouslyPressed = true;

    @Override
    public void periodic() {
        if (pushButton.getState() == false && ActiveButtonReset == 0){
            right_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ActiveButtonReset = 1;
            setTarget(-OFFSET);
        } else if (ActiveButtonReset == 1 && pushButton.getState() == true){
            ActiveButtonReset = 0;
            right_Motor.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
            left_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
//        boolean isLimitSwitchPressed = limitSwitch.getState();
//
//        if (isLimitSwitchPressed && !limitSwitchPreviouslyPressed) {
//            int encoderAtLimitSwitch = left_Motor.getCurrentPosition();
//            currentOffset = expectedZeroPosition - encoderAtLimitSwitch;
//
//            left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            left_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            limitSwitchPreviouslyPressed = true;
//        } else if (!isLimitSwitchPressed) {
//            limitSwitchPreviouslyPressed = false;
//        }

        if (target > getPosition()){
            armPID.setP(pUpper);
            armPID.setD(dUpper);
        } else if (target < getPosition()) {
            armPID.setP(pLower);
            armPID.setD(dLower);
        }

        double adjustedEncoderPosition = getPosition() + currentOffset;
        double motorOutput = armPID.calculate(adjustedEncoderPosition, target);

        left_Motor.setPower(motorOutput + armFeedForward(adjustedEncoderPosition));
        right_Motor.setPower(motorOutput + armFeedForward(adjustedEncoderPosition));
//
//        telemetry.addData("Output", motorOutput);
//        telemetry.addData("Target", target);
//        telemetry.addData("Current Angle", getPosition());
//        telemetry.addData("Feedforward", armFeedForward(adjustedEncoderPosition));
//        telemetry.addData("Error", armPID.getPositionError());
//        telemetry.addData("At Setpoint", armPID.atSetPoint());
//
//        telemetry.update();
    }

}