package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SetBlue extends OpMode {

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        Color.currentColor = Color.BLUE;
        requestOpModeStop();
    }
}
