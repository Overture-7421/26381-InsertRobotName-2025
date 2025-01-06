package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SGrabber extends SubsystemBase {

    // Motor Declaration
    private ServoEx sgrabber_Servo; // Declare left claw servo


    public SGrabber (HardwareMap hardwareMap) {
        //Servos IDs
        sgrabber_Servo = new SimpleServo(hardwareMap, "sgrabber_Servo", 0, 1.0);
    }

    public void setSgrabber_Position(double Sgrabber_Position) {
        sgrabber_Servo.setPosition(Sgrabber_Position);

    }

}