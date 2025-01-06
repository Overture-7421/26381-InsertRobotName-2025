package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Button;

import org.firstinspires.ftc.teamcode.Commands.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.MoveSGrabber;
import org.firstinspires.ftc.teamcode.Commands.MoveWrist;
import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Commands.ElevatorPositions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.SGrabber;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp
public class MainSystem extends LinearOpMode {
    @Override
    public void runOpMode() {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        /* SUBSYSTEM DECLARATION */
            Chassis chassis = new Chassis(hardwareMap);
            Intake intake = new Intake(hardwareMap);
            Wrist wrist = new Wrist(hardwareMap);
            SGrabber sGrabber = new SGrabber(hardwareMap);
            Arm arm = new Arm(hardwareMap);
            Elevator elevator = new Elevator(hardwareMap);

        /* GAMEPAD DECLARATION */
            GamepadEx driver = new GamepadEx(gamepad1);
            GamepadEx operator = new GamepadEx(gamepad2);

        /* COMMAND DECLARATION */
            // CHASSIS
            chassis.setDefaultCommand(new Drive(chassis, gamepad1));

            // INTAKE
            Button driverButtonX = driver.getGamepadButton(GamepadKeys.Button.X);
            driverButtonX.whenHeld(new MoveIntake(intake, 1.0));
            driverButtonX.whenReleased(new MoveIntake(intake, 0.0));

            Button driverButtonA = driver.getGamepadButton(GamepadKeys.Button.A);
            driverButtonA.whenHeld(new MoveIntake(intake, -1.0));
            driverButtonA.whenReleased(new MoveIntake(intake, 0.0));

            // WRIST
            Button operatorButtonY = operator.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenPressed(new MoveWrist(wrist, 0.0));
            operatorButtonY.whenReleased(new MoveWrist(wrist, 0.0));

            Button operatorButtonB = operator.getGamepadButton(GamepadKeys.Button.B);
            operatorButtonB.whenPressed(new MoveWrist(wrist, 0.5));
            operatorButtonB.whenReleased(new MoveWrist(wrist, 0.0));

            // GRABBER
            Button operatorButtonDPadUp = operator.getGamepadButton(GamepadKeys.Button.DPAD_UP);
            operatorButtonDPadUp.whenHeld(new MoveSGrabber(sGrabber, 1.0));
            operatorButtonDPadUp.whenReleased(new MoveSGrabber(sGrabber, 0.0));

            // ELEVATOR
            Button operatorRightBumper= operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            operatorRightBumper.whenPressed(new ElevatorPositions(elevator,60.0));

            Button operatorLeftBumper= operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
            operatorLeftBumper.whenPressed(new ElevatorPositions(elevator,0.0));

        waitForStart();
        chassis.reset(new Pose2d(0,0, Rotation2d.fromDegrees(0)));

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            Pose2d pose = chassis.getPose();

            // -- ODOMETRY TELEMETRY -- //
                telemetry.addData("X", pose.getX()); //This will display the telemetry on the DriverHub
                telemetry.addData("Y", pose.getY());
                telemetry.addData("Heading", pose.getRotation().getDegrees());
                telemetry.addData("RightDistance", chassis.rightDistance());
                telemetry.addData("LeftDistance", chassis.leftDistance());
                telemetry.addData("Elevator_Distance", elevator.getHeight());

            // -- UPDATE TELEMETRY -- //
                telemetry.update();
        }
    }
}





