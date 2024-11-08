package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;

@TeleOp
public class RealTest extends OpMode {
    Servo servo;
    DcMotor motor;
    InputManager inp;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "extendoWrist");
        motor = hardwareMap.get(DcMotor.class, "liftMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(2000);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        inp = new InputManager();
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.a, () -> servoPos += 0.05));
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.b, () -> servoPos -= 0.05));
    }

    double servoPos = 0;

    @Override
    public void loop() {
        inp.update();
        // servo.setPosition(servoPos);
        if (gamepad1.dpad_up) {
            motor.setPower(1);
        } else if (gamepad1.dpad_down) {
            motor.setPower(-.3);
        } else {
            motor.setPower(0);
        }
        telemetry.addLine("motor pos " + motor.getCurrentPosition());
        telemetry.addLine("motor set pos " + motor.getTargetPosition());
        telemetry.addData("servo pos", servo.getPosition());
        telemetry.addData("servopos", servoPos);
    }
}
