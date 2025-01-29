package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveToAprilTagOmni;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.RamsetteCommand;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.TurnToAngle;
import org.firstinspires.ftc.teamcode.Commands.Baskets.LowBasket;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GroundGrabLong;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.util.Arrays;


@Autonomous
public class GrabThreeSpecimensTwo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis = new Chassis(hardwareMap);
        Elevator elevator= new Elevator(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);


        //Forward
        TrajectoryConfig ForwardConfig = new TrajectoryConfig(0.5,0.2);
        ForwardConfig.setReversed(false);

        //Backward
        TrajectoryConfig BackwardConfig = new TrajectoryConfig(0.5,0.2);
        BackwardConfig.setReversed(true);

        Trajectory First = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(1.5,-0.6,Rotation2d.fromDegrees(0))), ForwardConfig
        );

       /* Trajectory Second = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.55,0.55,Rotation2d.fromDegrees(90)),
                new Pose2d(0.6,1.3,Rotation2d.fromDegrees(90))), ForwardConfig
        );

        Trajectory Third = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.55,1.3,Rotation2d.fromDegrees(90)),
                new Pose2d(0.9,1.3,Rotation2d.fromDegrees(-90))), ForwardConfig
        );*/

        Trajectory Second = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.5,-0.6,Rotation2d.fromDegrees(0)),
                new Pose2d(0.3,-0.7,Rotation2d.fromDegrees(0))), BackwardConfig
        );

        Trajectory Third = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.3,-0.7,Rotation2d.fromDegrees(0)),
                new Pose2d(1.5,-0.8,Rotation2d.fromDegrees(0))), ForwardConfig
        );
        Trajectory Four = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.5,-0.8,Rotation2d.fromDegrees(0)),
                new Pose2d(0.3,-0.9,Rotation2d.fromDegrees(0))), BackwardConfig
        );

        Trajectory Five = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.3,-0.9,Rotation2d.fromDegrees(0)),
                new Pose2d(1.5,-1,Rotation2d.fromDegrees(0))), ForwardConfig
        );

        Trajectory Six = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.5,-1,Rotation2d.fromDegrees(0)),
                new Pose2d(0.1,-1.1,Rotation2d.fromDegrees(0))), BackwardConfig
        );


        SequentialCommandGroup FirstCommandGroup = new SequentialCommandGroup(

                new RamsetteCommand(chassis, First),
                /*new RamsetteCommand(chassis, Second),
                new RamsetteCommand(chassis, Third),*/
                new RamsetteCommand(chassis, Second),
                new RamsetteCommand(chassis, Third),
                //new RamsetteCommand(chassis, Fifth)
                new RamsetteCommand(chassis, Four),
                new RamsetteCommand(chassis, Five),
                new RamsetteCommand(chassis, Six)


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