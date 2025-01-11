package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Button;

import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Baskets.HighBasket;
import org.firstinspires.ftc.teamcode.Commands.Baskets.LowBasket;
import org.firstinspires.ftc.teamcode.Commands.Chambers.HighChamber;
import org.firstinspires.ftc.teamcode.Commands.Chambers.LowChamber;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.MoveSGrabber;
import org.firstinspires.ftc.teamcode.Commands.MoveWrist;
import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ElevatorPositions;

import org.firstinspires.ftc.teamcode.Commands.StowAll;
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
        //telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

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

            Button driverButtonB = driver.getGamepadButton(GamepadKeys.Button.B);
            driverButtonB.whenHeld(new MoveIntake(intake, -1.0));
            driverButtonB.whenReleased(new MoveIntake(intake, 0.0));

            //GroundGrab
            Button driverButtonRightBumper = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            driverButtonRightBumper.whenPressed(new GroundGrab(arm, elevator, wrist));

            // GRABBER
            //Button operatorButtonLeftBumper = operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
            //operatorButtonLeftBumper.whenHeld(new MoveSGrabber(sGrabber, 0.2));
            //operatorButtonLeftBumper.whenReleased(new MoveSGrabber(sGrabber, 0.0));

            // Baskets
            Button operatorBButton= operator.getGamepadButton(GamepadKeys.Button.B);
            operatorBButton.whenPressed(new HighBasket(arm, elevator, wrist));

            Button operatorAButton= operator.getGamepadButton(GamepadKeys.Button.A);
            operatorAButton.whenPressed(new LowBasket(arm, elevator, wrist));

            //Chambers
            Button operatorButtonX = operator.getGamepadButton(GamepadKeys.Button.X);
            operatorButtonX.whenPressed(new LowChamber(arm, elevator, wrist));

            Button operatorButtonY = operator.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenPressed(new HighChamber(arm, elevator, wrist));

            //Button operatorButtonRightBumper = operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            //operatorButtonRightBumper.whenPressed(new StowAll(arm, elevator, wrist));

            Button operatorButtonLeftBumper = operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
            operatorButtonLeftBumper.whenPressed(new MoveWrist(wrist, 0.0));

            Button operatorButtonLeftBumpe = operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            operatorButtonLeftBumpe.whenPressed(new MoveWrist(wrist, 0.6));




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
                telemetry.addLine("---- Medidas ---");
                telemetry.addData("Elevator Distance", elevator.getHeight());
                telemetry.addData("Arm Position", arm.getPosition());
                telemetry.addData("Right Servo:", wrist.getRightServoPosition());
                telemetry.addData("Left Servo:", wrist.getLeftServoPosition());

            // -- UPDATE TELEMETRY -- //
                telemetry.update();
        }
    }
}





