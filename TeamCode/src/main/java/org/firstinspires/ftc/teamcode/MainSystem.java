package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
    public void runOpMode(){

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry( FtcDashboard.getInstance().getTelemetry());

        Chassis chassis = new Chassis(hardwareMap); //Here you can add every element of the robot
        //Intake intake = new Intake(hardwareMap);
        //Wrist wrist = new Wrist(hardwareMap);
        //SGrabber sGrabber = new SGrabber(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        //Elevator elevator = new Elevator(hardwareMap);
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);


// -------------------------------------------------------------- //
// -------------------------------------------------------------- //

       // chassis.setDefaultCommand(new Drive(chassis,gamepad1));


        /*Button operatorButtonX= operator.getGamepadButton(GamepadKeys.Button.X);
        operatorButtonX.whenHeld(new MoveIntake(intake,1.0));
        operatorButtonX.whenReleased(new MoveIntake(intake,0.0));

        Button operatorButtonA= operator.getGamepadButton(GamepadKeys.Button.A);
        operatorButtonA.whenHeld(new MoveIntake(intake,-1.0));
        operatorButtonA.whenReleased(new MoveIntake(intake,0.0));*/


        /*Button operatorButtonA= operator.getGamepadButton(GamepadKeys.Button.A);
        operatorButtonA.whenPressed(new MoveWrist(wrist,0.0));
        operatorButtonA.whenReleased(new MoveWrist(wrist,0.0));*/

        /*Button operatorButtonB= operator.getGamepadButton(GamepadKeys.Button.B);
        operatorButtonB.whenPressed(new MoveWrist(wrist,0.5));
        operatorButtonB.whenReleased(new MoveWrist(wrist,0.0));*/






        /*Button operatorButtonDPadUp= operator.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        operatorButtonDPadUp.whenHeld(new MoveSGrabber(sGrabber,1.0));
        operatorButtonDPadUp.whenReleased(new MoveSGrabber(sGrabber,0.0));


        Button operatorButtonY= operator.getGamepadButton(GamepadKeys.Button.Y);
        operatorButtonY.whenHeld(new ElevatorPositions(elevator,40));
        operatorButtonY.whenReleased(new ElevatorPositions(elevator,10));*/

        /*Button operatorButtonY= operator.getGamepadButton(GamepadKeys.Button.Y);
        operatorButtonY.whenPressed(new ElevatorPositions(elevator,60.0));

        Button operatorButtonA= operator.getGamepadButton(GamepadKeys.Button.A);
        operatorButtonA.whenPressed(new ElevatorPositions(elevator,0.0));*/





        waitForStart();
        chassis.reset(new Pose2d(0,0, Rotation2d.fromDegrees(0)));

        while (opModeIsActive()) { //This will occur whenever the op mode is active
            CommandScheduler.getInstance().run();
            Pose2d pose = chassis.getPose();

            // -- ODOMETRY TELEMETRY -- //
               /* telemetry.addData("X", pose.getX()); //This will display the telemetry on the DriverHub
                telemetry.addData("Y", pose.getY());
                telemetry.addData("Heading", pose.getRotation().getDegrees());
                telemetry.addData("RightDistance", chassis.rightDistance());
                telemetry.addData("LeftDistance", chassis.leftDistance());
                telemetry.addData("Potentiometer voltage", arm.getVoltage());*/
            //telemetry.addData("Elevator_Distance", elevator.getHeight());
            //telemetry.addData("Maxvoltage", arm.getPointometerMaxVoltage());
            // -- UPDATE TELEMETRY -- //
            telemetry.update();
        }
    }
}





