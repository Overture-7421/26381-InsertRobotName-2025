package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.ChassisPaths;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.RamsetteCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Commands.Baskets.LowBasket;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.util.Arrays;


@Autonomous
public class LowBasketAndMove extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis = new Chassis(hardwareMap);
        Elevator elevator= new Elevator(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        //Forward
        TrajectoryConfig ForwardConfig = new TrajectoryConfig(0.5,0.2);
        ForwardConfig.setReversed(false);

        //Backward
        TrajectoryConfig BackwardConfig = new TrajectoryConfig(0.6,0.3);
        BackwardConfig.setReversed(true);

        Trajectory First = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(1.45,-0.85,Rotation2d.fromDegrees(0))), ForwardConfig
        );



        SequentialCommandGroup FirstCommandGroup = new SequentialCommandGroup(
                new MoveIntake(intake, Constants.Intake.INTAKE_STOW),
                new LowBasket(arm, elevator, wrist),
                new ChassisPaths(chassis, 0.0, 0.25).withTimeout(2000),
                new MoveIntake(intake, Constants.Intake.INTAKE_OPEN),
                new WaitCommand(2000),
                new ChassisPaths(chassis, 0.0, -0.2).withTimeout(2000),
                new WaitCommand(2000),
                new StowAll(arm, elevator, wrist),
                new WaitCommand(1000)
        );


        waitForStart();
        chassis.reset(new Pose2d());
        CommandScheduler.getInstance().schedule(FirstCommandGroup);

        while (opModeIsActive ()){
            CommandScheduler.getInstance().run();

            Pose2d pose = chassis.getPose();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getRotation().getDegrees());

            telemetry.addLine("--- Chassis Telemetry ---");
            telemetry.addData("RightDistance", chassis.rightDistance());
            telemetry.addData("LeftDistance", chassis.leftDistance());
            telemetry.update();
        }
    }
}