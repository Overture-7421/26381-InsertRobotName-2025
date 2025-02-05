package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class WaitForButton extends CommandBase {

    private final GamepadEx gamepad;
    private final GamepadKeys.Button button;

    public WaitForButton(GamepadEx gamepad, GamepadKeys.Button button){
        this.gamepad = gamepad;
        this.button = button;
    }

    @Override
    public boolean isFinished() {
        return gamepad.getButton(button);
    }
}