package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist extends SubsystemBase {

    // Motor Declaration
    private final ServoEx wrist_RightServo; // Declare left claw servo
    private final ServoEx wrist_LeftServo; //Declare right claw servo

    public Wrist (HardwareMap hardwareMap) {
        //Servos IDs
        wrist_RightServo = new SimpleServo(hardwareMap, "wrist_RightServo", 0, 1.0);
        wrist_LeftServo = new SimpleServo(hardwareMap, "wrist_LeftServo", 0, 1.0);
        wrist_LeftServo.setInverted(false);
        wrist_RightServo.setInverted(true);
    }

    public void wristPosition(double wristMotorsPosition) {
        wrist_RightServo.setPosition(wristMotorsPosition);
        wrist_LeftServo.setPosition(wristMotorsPosition);

    }

    public double getRightServoPosition(){
        double right = wrist_RightServo.getAngle();
        return (right);
    }

    public double getLeftServoPosition(){
        double left = wrist_LeftServo.getPosition();
        return (left);
    }

}