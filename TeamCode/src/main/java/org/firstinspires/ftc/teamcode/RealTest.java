package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RealTest extends OpMode {
    Servo servo;
    DcMotor motor;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    double servoPos = 0;
    @Override
    public void loop() {
        if (gamepad1.a) {
            servo.setPosition(servoPos += 0.05);
        } else if (gamepad1.b) {
            servo.setPosition(servoPos -= 0.05);
        }
        if (gamepad1.dpad_up) {
            motor.setPower(.1);
        } else if (gamepad1.dpad_down) {
            motor.setPower(-.1);
        }
        telemetry.addData("motor pos", motor.getCurrentPosition());
    }
}
