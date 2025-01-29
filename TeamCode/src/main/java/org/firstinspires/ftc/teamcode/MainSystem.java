package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Button;

import org.firstinspires.ftc.teamcode.Commands.Arm.ModifyArmCommand;
import org.firstinspires.ftc.teamcode.Commands.Baskets.HighBasket;
import org.firstinspires.ftc.teamcode.Commands.Baskets.LowBasket;
import org.firstinspires.ftc.teamcode.Commands.Chambers.HighChamber;
import org.firstinspires.ftc.teamcode.Commands.Chambers.LowChamber;
import org.firstinspires.ftc.teamcode.Commands.Climber.Climb;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ModifyElevatorCommand;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GroundGrabLong;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GroundGrabMedium;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GroundGrabShort;
import org.firstinspires.ftc.teamcode.Commands.Intake.MoveIntake;
import org.firstinspires.ftc.teamcode.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Commands.Drive;

import org.firstinspires.ftc.teamcode.Commands.StowAll;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

@TeleOp
public class MainSystem extends LinearOpMode {

    private ModifyArmCommand modifyArmCommand;
    private ModifyElevatorCommand modifyElevatorCommand;

    @Override
    public void runOpMode() {

        /* SUBSYSTEM DECLARATION */
            Chassis chassis = new Chassis(hardwareMap);
            Intake intake = new Intake(hardwareMap);
            Wrist wrist = new Wrist(hardwareMap);
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

            // GROUND GRAB COMMANDS
                    // SHORT GROUND GRAB
                    Button driverButtonA = driver.getGamepadButton(GamepadKeys.Button.A);
                    driverButtonA.whenHeld(new GroundGrabShort(arm, elevator,wrist));
                    driverButtonA.whenReleased(new GroundGrabShort(arm, elevator, wrist));

                    // MEDIUM GROUND GRAB
                    Button driverButtonY = driver.getGamepadButton(GamepadKeys.Button.Y);
                    driverButtonY.whenHeld(new GroundGrabMedium(arm, elevator, wrist));
                    driverButtonY.whenReleased(new GroundGrabMedium(arm, elevator, wrist));

                    // LONG GROUND GRAB
                    Button driverRightBumper = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
                    driverRightBumper.whenHeld(new GroundGrabLong(arm, elevator, wrist));
                    driverRightBumper.whenReleased(new GroundGrabLong(arm, elevator, wrist));

            // MANUAL WRIST
            Button operatorDPadLeft = operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
            operatorDPadLeft.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_MEDIUM));

            Button operatorDPadDown = operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
            operatorDPadDown.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_STOW));

            Button operatorDPadRight = operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
            operatorDPadRight.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_LONG));

            Button operatorDPadUp = operator.getGamepadButton(GamepadKeys.Button.DPAD_UP);
            operatorDPadUp.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_SHORT));

            // MANUAL ARM
            modifyArmCommand = new ModifyArmCommand(arm);
            modifyArmCommand.setGamepad(gamepad1);
            arm.setDefaultCommand(modifyArmCommand);

            // MANUAL ELEVATOR
            modifyElevatorCommand = new ModifyElevatorCommand(elevator);
            modifyElevatorCommand.setGamepad(gamepad1);
            elevator.setDefaultCommand(modifyElevatorCommand);

        /* GAME ROUTINES */
            // BASKETS
            Button operatorBButton= operator.getGamepadButton(GamepadKeys.Button.B);
            operatorBButton.whenPressed(new HighBasket(arm, elevator, wrist));

            Button operatorAButton= operator.getGamepadButton(GamepadKeys.Button.A);
            operatorAButton.whenPressed(new LowBasket(arm, elevator, wrist));

            // CHAMBERS
            Button operatorButtonX = operator.getGamepadButton(GamepadKeys.Button.X);
            operatorButtonX.whenPressed(new LowChamber(arm, elevator, wrist));

            Button operatorButtonY = operator.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenPressed(new HighChamber(arm, elevator, wrist));

            // STOW ALL
            Button operatorRightBumper = operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            operatorRightBumper.whenPressed(new StowAll(arm, elevator, wrist));

            // CLIMB
            Button driverLeftBumper = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
            driverLeftBumper.whenPressed(new Climb(arm, elevator, wrist));

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
                telemetry.addLine("---- MEASUREMENTS ---");
                telemetry.addData("Elevator Distance", elevator.getHeight());
                telemetry.addData("Arm Position", arm.getPosition());
                telemetry.addData("Right Servo:", wrist.getRightServoPosition());
                telemetry.addData("Left Servo:", wrist.getLeftServoPosition());

            // -- UPDATE TELEMETRY -- //
                telemetry.update();
        }
    }
}





