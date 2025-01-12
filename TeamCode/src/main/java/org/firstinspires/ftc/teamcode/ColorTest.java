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
    private DistanceSensor sensor;

    @Override
    public void init() {
        sensor = hardwareMap.get(DistanceSensor.class, "extendoDist");
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
        telemetry.addData("dist cm", sensor.getDistance(DistanceUnit.CM));
    }
}
