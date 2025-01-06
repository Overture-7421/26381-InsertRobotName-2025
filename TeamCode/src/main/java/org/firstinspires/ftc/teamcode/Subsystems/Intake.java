package org.firstinspires.ftc.teamcode.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake extends SubsystemBase {

    private CRServo intake_Servo;

    public Intake(HardwareMap hardwareMap){
        intake_Servo = hardwareMap.get(CRServo.class, "grabServo");
        intake_Servo.setDirection(CRServo.Direction.FORWARD);
    }

    // Method to set the servo's speed
    public void setSpeed(double speed) {
        intake_Servo.setPower(speed); // Speed ranges from -1.0 to 1.0
    }

    // Method to stop the servo
    public void stop() {
        intake_Servo.setPower(0);
    }

}
