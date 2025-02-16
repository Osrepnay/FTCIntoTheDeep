package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;

@Config
@TeleOp
public class ColorTest extends OpMode {
    InputManager inp;
    Drivetrain dt;
    private ColorSensor sensor;

    @Override
    public void init() {
        sensor = hardwareMap.get(ColorSensor.class, "extendoDist");
        dt = new Drivetrain(hardwareMap);
        inp = new InputManager();
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.a, () -> servoPos += 0.05));
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.b, () -> servoPos -= 0.05));
    }

    double servoPos = 0;

    @Override
    public void loop() {
        inp.update();
        dt.setPowers(dt.mix(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
        telemetry.addData("dist cm", ((DistanceSensor) sensor).getDistance(DistanceUnit.CM));
        telemetry.addData("dist opt", ((OpticalDistanceSensor) sensor).getLightDetected());
        telemetry.addData("alpha", sensor.alpha());
        telemetry.addData("red", sensor.red());
        telemetry.addData("green", sensor.green());
        telemetry.addData("blue", sensor.blue());
        int argb = sensor.argb();
        int alpha = (argb & 0xFF000000) >>> (6 * 4);
        int red   = (argb & 0x00FF0000) >>> (4 * 4);
        int green = (argb & 0x0000FF00) >>> (2 * 4);
        int blue  = (argb & 0x000000FF);
        telemetry.addData("alpha argb", alpha);
        telemetry.addData("red argb", red);
        telemetry.addData("green argb", green);
        telemetry.addData("blue argb", blue);
        telemetry.addData("argb bin", Integer.toBinaryString(argb));
    }
}
