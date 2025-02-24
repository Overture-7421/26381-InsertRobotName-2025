package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.RamsetteCommand;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.TurnToAngle;
import org.firstinspires.ftc.teamcode.Commands.Baskets.AutoLowBasket;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.AutoGroundGrab;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.util.Arrays;


@Autonomous
public class LowBasketAndPark extends LinearOpMode {

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
        TrajectoryConfig ForwardConfig = new TrajectoryConfig(0.7,0.4);
        ForwardConfig.setReversed(false);

        //Backward
        TrajectoryConfig BackwardConfig = new TrajectoryConfig(0.6,0.3);
        BackwardConfig.setReversed(true);

        Trajectory First = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                new Pose2d(0.0,0,Rotation2d.fromDegrees(75)),
                new Pose2d(0.15,0.52,Rotation2d.fromDegrees(75))), ForwardConfig
        );

        SequentialCommandGroup FirstCommandGroup = new SequentialCommandGroup(
                new RamsetteCommand(chassis, First),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(135)),
                new AutoLowBasket(arm, elevator, wrist, intake),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)),
                new AutoGroundGrab(arm, elevator, wrist, intake).withTimeout(1500),
                new MoveIntake(intake, Constants.Intake.INTAKE_STOW),
                new StowAll(arm, elevator, wrist),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(135)),
                new AutoLowBasket(arm, elevator, wrist, intake)
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