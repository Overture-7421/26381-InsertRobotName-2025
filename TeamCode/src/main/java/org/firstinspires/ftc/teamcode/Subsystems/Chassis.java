package org.firstinspires.ftc.teamcode.Subsystems;
/*
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.overture.ftc.overftclib.Contollers.PIDController;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;

public class Chassis extends SubsystemBase {
    // Motors Declaration
    private DcMotor rightDrive;
    private DcMotor leftDrive;

    private final double M_PER_TICK = (28); //Motor (Doesn't change)
    static final double TRACKWIDTH = 0.0891286; // Depends on the wheels
    static final double GEAR_REDUCTION = 20; // Depends on the motor configuration

    private DifferentialDriveOdometry diffOdom;

    private IMU imu;
    private int leftOffset = 0, rightOffset = 0;

    public Chassis(HardwareMap hardwareMap) {
        // Motor ID
        rightDrive = (DcMotor) hardwareMap.get(DcMotor.class, "right_Drive");
        leftDrive = (DcMotor) hardwareMap.get(DcMotor.class, "left_Drive");

        // Invert one motor
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);

        // Odometry initialization
        diffOdom = new DifferentialDriveOdometry(new Rotation2d());
        imu = hardwareMap.get(IMU.class, "imu");


        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        );

        imu.initialize(imuParameters);
        imu.resetYaw();

    }

    // Set Speed Function
    public void setSpeed(double linearSpeed, double angularSpeed){
        rightDrive.setPower(linearSpeed - angularSpeed);
        leftDrive.setPower(linearSpeed + angularSpeed);
    }

    // Get Right Distance (Position)
    public double rightDistance(){
        return ((rightDrive.getCurrentPosition() / M_PER_TICK) * TRACKWIDTH * Math.PI) / GEAR_REDUCTION;
    }

    // Get Left Distance (Position)
    public double leftDistance(){
        return ((leftDrive.getCurrentPosition() / M_PER_TICK) * TRACKWIDTH * Math.PI) / GEAR_REDUCTION;
    }

    public void resetPose(Pose2d pose) {
        leftOffset = leftDrive.getCurrentPosition();
        rightOffset = rightDrive.getCurrentPosition();
        diffOdom.resetPosition(pose, getIMUHeading());
    }

    public Pose2d getPose() {
        return diffOdom.getPoseMeters();
    }

    @Override
    public void periodic() {
        diffOdom.update(getIMUHeading(), leftDistance(), rightDistance());
    }

    private Rotation2d getIMUHeading(){
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        return Rotation2d.fromDegrees(robotOrientation.getYaw(AngleUnit.DEGREES));
    }
}

 */