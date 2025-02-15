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
import org.firstinspires.ftc.teamcode.Commands.Arm.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.Baskets.HighBasket;
import org.firstinspires.ftc.teamcode.Commands.Baskets.LowBasket;
import org.firstinspires.ftc.teamcode.Commands.Chambers.HighChamber;
import org.firstinspires.ftc.teamcode.Commands.Chambers.LowChamber;
import org.firstinspires.ftc.teamcode.Commands.Climber.Climb;
import org.firstinspires.ftc.teamcode.Commands.Elevator.ModifyElevatorCommand;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GrabSpecimens;

import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GrabSpecimensOrientation;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GroundGrabHover;
import org.firstinspires.ftc.teamcode.Commands.GroundGrab.GroundGrabPick;

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
        CommandScheduler.getInstance().reset();

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
            driverButtonX.whenPressed(new MoveIntake(intake, 0.1));

            Button driverButtonB = driver.getGamepadButton(GamepadKeys.Button.B);
            driverButtonB.whenPressed(new MoveIntake(intake, 0.5));

            // GRAB
            Button driverRightBumper = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            driverRightBumper.whenPressed(new GroundGrabHover(intake, arm, elevator, wrist, driver, operator));

            Button driverAButton = driver.getGamepadButton(GamepadKeys.Button.A);
            driverAButton.whenPressed(new GroundGrabPick(intake, arm, elevator, wrist, driver, operator));

        // MANUAL WRIST
            Button driverDPadLeft = driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
            driverDPadLeft.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_EXTEND_SHORT));

            Button driverDPadUp = driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
            driverDPadUp.whenPressed(new MoveWrist(wrist, Constants.Wrist.WRIST_STOW));

        // GRAB SPECIMENS
            Button operatorLeftBumper= operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
            operatorLeftBumper.whenPressed(new GrabSpecimens(arm, operator, wrist, intake, elevator));

        // GRAB SPECIMENS ORIENTATION
            Button operatorStartButton = operator.getGamepadButton(GamepadKeys.Button.START);
            operatorStartButton.whenPressed(new GrabSpecimensOrientation(arm, operator, wrist, intake, elevator));

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
            Button operatorAButton= operator.getGamepadButton(GamepadKeys.Button.A);
            operatorAButton.whenHeld(new LowBasket(arm, elevator, wrist));
            operatorAButton.whenReleased(new StowAll(arm, elevator, wrist));

            Button operatorBButton = operator.getGamepadButton(GamepadKeys.Button.B);
            operatorBButton.whenHeld(new HighBasket(arm, elevator, wrist));
            operatorBButton.whenReleased(new StowAll(arm, elevator, wrist));

            // CHAMBERS
            Button operatorButtonX = operator.getGamepadButton(GamepadKeys.Button.X);
            operatorButtonX.whenPressed(new LowChamber(arm, elevator, wrist, operator, intake));

            Button operatorButtonY = operator.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenPressed(new HighChamber(arm, elevator, wrist, intake, operator));
      
            operatorButtonX.whenHeld(new LowChamber(arm, elevator, wrist, operator));
            operatorButtonX.whenReleased(new StowAll(arm, elevator, wrist));

            Button operatorButtonY = operator.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenHeld(new HighChamber(arm, elevator, wrist, intake));
            operatorButtonY.whenReleased(new StowAll(arm, elevator, wrist));

            // STOW ALL
            Button operatorRightBumper = operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
            operatorRightBumper.whenPressed(new StowAll(arm, elevator, wrist));

            // CLIMB
            Button operatorDpadUp= operator.getGamepadButton(GamepadKeys.Button.DPAD_UP);
            operatorDpadUp.whenPressed(new Climb(arm, elevator, wrist, operator));

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
                telemetry.addData("Elevator Target", elevator.target);
                telemetry.addData("Arm Position", arm.getPosition());
                telemetry.addData("Right Servo:", wrist.getRightServoPosition());
                telemetry.addData("Left Servo:", wrist.getLeftServoPosition());

                telemetry.addData(" LIMIT SWITCH PRESSED", arm.isLimitReached());

            // -- UPDATE TELEMETRY -- //
                telemetry.update();
        }
    }
}





