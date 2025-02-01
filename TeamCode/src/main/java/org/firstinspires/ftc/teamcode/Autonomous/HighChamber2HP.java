package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
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
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Baskets.LowBasket;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GroundGrabLong;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;

import java.util.Arrays;


@Autonomous
public class HighChamber2HP extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis = new Chassis(hardwareMap);
        Elevator elevator= new Elevator(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);


        //Forward
        TrajectoryConfig ForwardConfig = new TrajectoryConfig(0.6,0.3);
        ForwardConfig.setReversed(false); //Vel 0.5 //Accel 0.2

        //Backward
        TrajectoryConfig BackwardConfig = new TrajectoryConfig(0.7,0.4);
        BackwardConfig.setReversed(true); //Vel 0.5 //Accel 0.2

        Trajectory ForChamber = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(0.65,0.0,Rotation2d.fromDegrees(0))), ForwardConfig
        );

        Trajectory ReChamber = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.65,0.0,Rotation2d.fromDegrees(0)),
                new Pose2d(0.0,0.0,Rotation2d.fromDegrees(0))), BackwardConfig
        );

        Trajectory First = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.0,0,Rotation2d.fromDegrees(0)),
                new Pose2d(1.45,-0.80,Rotation2d.fromDegrees(0))), ForwardConfig
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
                new Pose2d(1.45,-0.80,Rotation2d.fromDegrees(0)),
                new Pose2d(1.25,-0.80,Rotation2d.fromDegrees(10)),
                new Pose2d(0.0,-0.89,Rotation2d.fromDegrees(0))), BackwardConfig
        );

        Trajectory Third = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.0,-0.89,Rotation2d.fromDegrees(0)),
                new Pose2d(1.45,-0.93,Rotation2d.fromDegrees(0))), ForwardConfig
        );
        Trajectory Four = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.45,-0.93 ,Rotation2d.fromDegrees(0)),
                new Pose2d(1.25,-0.95,Rotation2d.fromDegrees(10)),
                new Pose2d(0.07,-1.14,Rotation2d.fromDegrees(0))), BackwardConfig
        );

        Trajectory Five = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.07,-1.14,Rotation2d.fromDegrees(0)),
                new Pose2d(1.45,-1.14,Rotation2d.fromDegrees(0))), ForwardConfig
        );

        Trajectory Six = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(1.45,-1.34,Rotation2d.fromDegrees(0)),
                new Pose2d(1.25,-1.37,Rotation2d.fromDegrees(10)),
                new Pose2d(0.1,-1.37,Rotation2d.fromDegrees(0))), BackwardConfig
        );


        SequentialCommandGroup FirstCommandGroup = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new RamsetteCommand(chassis, ForChamber),
                        new MoveArm(arm, Constants.Arm.ARM_HIGHCHAMBER),
                        new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_HIGHCHAMBER)
                        ).withTimeout(3000),

                new ParallelCommandGroup(
                        new RamsetteCommand(chassis, ReChamber),
                        new ElevatorPositions(elevator, Constants.Elevator.ELEVATOR_STOW)
                        ).withTimeout(2000),
                new MoveArm(arm, Constants.Arm.ARM_STOW).withTimeout(500),
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