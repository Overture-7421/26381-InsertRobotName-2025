package org.firstinspires.ftc.teamcode.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake extends SubsystemBase {

    private CRServo intake_RightServo;
    private CRServo intake_LeftServo;

    public Intake(HardwareMap hardwareMap){
        intake_RightServo = hardwareMap.get(CRServo.class, "intakeRightServo");
        intake_RightServo.setDirection(CRServo.Direction.FORWARD);

        intake_LeftServo = hardwareMap.get(CRServo.class, "intakeLeftServo");
        intake_LeftServo.setDirection(CRServo.Direction.REVERSE);
    }

    public void setSpeed(double speed) {
        intake_RightServo.setPower(speed); // Speed ranges from -1.0 to 1.0
        intake_LeftServo.setPower(speed); // Speed ranges from -1.0 to 1.0
    }

    public void stop() {
        intake_RightServo.setPower(0);
        intake_LeftServo.setPower(0);
    }

}
