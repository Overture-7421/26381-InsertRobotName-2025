package org.firstinspires.ftc.teamcode.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;


public class Intake extends SubsystemBase {

    private ServoEx intakeServo;

    public Intake(HardwareMap hardwareMap){
    intakeServo = new SimpleServo(hardwareMap, "intake", 0.0, 1.0);
    }

    public void IntakePosition(double position) {
        intakeServo.setPosition(position); // Speed ranges from -1.0 to 1.0
    }


}
