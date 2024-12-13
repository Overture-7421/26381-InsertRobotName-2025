package org.firstinspires.ftc.teamcode.Subsystems;
import com.overture.ftc.overftclib.Devices.IOverDcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
import java.lang.Math;
public class Arm extends SubsystemBase {

    private DcMotorEx right_ArmMotor;
    private DcMotorEx left_ArmMotor;

    private ProfiledPIDController armPID;
    private AnalogInput arm_Potentiometer;

    public Arm (HardwareMap hardwareMap){

        right_ArmMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "right_ArmMotor");
        left_ArmMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "left_ArmMotor");

        arm_Potentiometer = (AnalogInput) hardwareMap.get(AnalogInput.class, "potentiometer");

        armPID = new ProfiledPIDController(0.1,0.0,0.0, new TrapezoidProfile.Constraints (3.0,2.0));

        right_ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        right_ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD );

    }



   /* public void resetZero(){
        ARM_OFFSET= arm_Motor.getCurrentPosition();
    }

    public double getPosition(){
        double currentTicks = arm_Motor.getCurrentPosition();
        double currentPosition = (currentTicks/COUNTS_PER_REV*GEAR_RATIO)-(ARM_OFFSET/360);
        return currentPosition;
    }

    public void setTarget(double targetPos){
        if (armPID.getGoal().position != targetPos){
            armPID.reset(getPosition());
            armPID.setGoal(targetPos);
        }
    }

    public void setTarget(double targetPosition) {
        double currentVoltage = potentiometer.getVoltage();
        armPID.calculate(currentVoltage, targetPosition);
    }*/

    public double getVoltage() {
        double currentPosition = arm_Potentiometer.getVoltage();
        return currentPosition;
    }

    public double getPointometerMaxVoltage() {
        double pointometerMaxVoltage = arm_Potentiometer.getMaxVoltage();
        return pointometerMaxVoltage;
    }

    public double getPotentiometerAngle(){

        double angle = arm_Potentiometer.getVoltage() * 81.8;
        return angle;

        /*
        double appliedVoltage =3.3;

        double PPosition = (getVoltage()/getPointometerMaxVoltage())*(270/appliedVoltage);
        return PPosition;
        */

    }

    public double setArmAngle(double armAngle) {
        double voltage = (445.5 * (armAngle-270.0)) / ((armAngle * armAngle) - (270 * armAngle) - 36450);
        return voltage;
    }

    /*
    public void setArmTarget(double DesiredPos ){
        double ArmTarget = DesiredPos;
        while (Math.abs(ArmTarget-getPPosition())>0.05){
            arm_Motor.setPower(1);
        }
        arm_Motor.setPower(0);
    }
    */

    @Override
    public void periodic(){
        double motorOutput = armPID.calculate(getVoltage());
        arm_Motor.setPower(motorOutput);
    }

}
