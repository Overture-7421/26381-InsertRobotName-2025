package org.firstinspires.ftc.teamcode.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Chassis extends SubsystemBase {
    private DcMotorEx left_DriveTrain;
    private DcMotorEx right_DriveTrain;

    private final double TICKS_PER_METER = 28.0;
    private final double TRACK_WIDTH = 0.09;
    private final double REDUCTION = 20.0;

    private DifferentialDriveOdometry differentialDriveOdometry;
    private IMU imu;
    private double leftOffset = 0.0;
    private double rightOffset = 0.0;

    public Chassis (HardwareMap hardwareMap){
        left_DriveTrain = (DcMotorEx) hardwareMap.get(DcMotor.class,"left_Drive");
        right_DriveTrain = (DcMotorEx) hardwareMap.get(DcMotor.class, "right_Drive");

        left_DriveTrain.setDirection(DcMotorEx.Direction.REVERSE);
        right_DriveTrain.setDirection(DcMotorEx.Direction.FORWARD);

        differentialDriveOdometry = new DifferentialDriveOdometry(new Rotation2d());
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParemeters = new IMU.Parameters(new
                RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(imuParemeters);
        imu.resetYaw();
    }

    public void setVelocity( double linearVelocity, double angularVelocity){
        left_DriveTrain.setPower(linearVelocity - angularVelocity);
        right_DriveTrain.setPower(linearVelocity + angularVelocity);

    }
    public double leftDistance() {
        return ((left_DriveTrain.getCurrentPosition() / TICKS_PER_METER) * TRACK_WIDTH * Math.PI) / REDUCTION;
    }
    public double rightDistance() {
        return ((right_DriveTrain.getCurrentPosition() / TICKS_PER_METER) * TRACK_WIDTH * Math.PI) / REDUCTION;
    }

    public void reset(Pose2d pose2d){
        leftOffset = left_DriveTrain.getCurrentPosition();
        rightOffset = right_DriveTrain.getCurrentPosition();
        differentialDriveOdometry.resetPosition(pose2d, getIMUHeading());
    }

    public Pose2d getPose(){
        return differentialDriveOdometry.getPoseMeters();
    }

    @Override
    public void periodic(){
        differentialDriveOdometry.update(getIMUHeading(),leftDistance(),rightDistance());
    }

    private Rotation2d getIMUHeading(){
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        return Rotation2d.fromDegrees(robotOrientation.getYaw(AngleUnit.DEGREES));
    }

}
