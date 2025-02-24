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
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.TurnToAngle;
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Baskets.AutoHighBasket;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.AutoGroundGrab;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.RamsetteCommand;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousIndex.TurnToAngle;
import org.firstinspires.ftc.teamcode.Commands.Baskets.AutoHighBasket;
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
public class HighBasketAndPark extends LinearOpMode{

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
                new Pose2d(0.0,0,Rotation2d.fromDegrees(45)),
                new Pose2d(0.39,0.39,Rotation2d.fromDegrees(45))), ForwardConfig
        );

        SequentialCommandGroup FirstCommandGroup = new SequentialCommandGroup(
                //preloaded piece
                new RamsetteCommand(chassis, First),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-39)),
                new AutoHighBasket(chassis, arm, elevator, wrist).withTimeout(2000),
                new ChassisPaths(chassis, 0.0, -0.4).withTimeout(900),
                new MoveIntake(intake, Constants.Intake.INTAKE_OPEN),
                new WaitCommand(500),
                new ChassisPaths(chassis, 0.0, 0.4).withTimeout(900),
                new StowAll(arm, elevator, wrist),

                //Rotate towards second piece
                new TurnToAngle(chassis, Rotation2d.fromDegrees(0)).withTimeout(2000),
                new ChassisPaths(chassis, 0.0, -0.4).withTimeout(540),

                //Second piece
                new AutoGroundGrab(arm, elevator, wrist, intake).withTimeout(2000),
                new MoveArm(arm, -2),
                new MoveIntake(intake, Constants.Intake.INTAKE_STOW),
                new StowAll(arm, elevator, wrist),
                new ChassisPaths(chassis, 0.0, 0.4).withTimeout(800),
                new TurnToAngle(chassis, Rotation2d.fromDegrees(-40)).withTimeout(2000),
                new AutoHighBasket(chassis, arm, elevator, wrist).withTimeout(2000),
                new ChassisPaths(chassis, 0.0, -0.4).withTimeout(920),
                new MoveIntake(intake, Constants.Intake.INTAKE_OPEN).withTimeout(500),
                new ChassisPaths(chassis, 0.0, 0.4).withTimeout(800),
                new StowAll(arm, elevator, wrist)
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

