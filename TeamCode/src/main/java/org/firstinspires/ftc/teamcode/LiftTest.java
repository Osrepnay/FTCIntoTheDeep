package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.noncents.PIDController;
import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;

@Config
@TeleOp
public class LiftTest extends OpMode {
    DcMotor left;
    DcMotor right;
    InputManager inp;
    Drivetrain dt;
    VoltageSensor voltageSensor;
    Telemetry dash;

    public static PIDController pid = new PIDController(0.008, 0.0001, 0.25, 1000);
    public static long TARGET = 0;
    public static double HOLD_0 = 0.05;
    public static double HOLD_1 = 0.1;
    public static double HOLD_2 = 0.15;

    @Override
    public void init() {
        dash = FtcDashboard.getInstance().getTelemetry();
        left = hardwareMap.get(DcMotor.class, "liftLeft");
        right = hardwareMap.get(DcMotor.class, "liftRight");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        dt = new Drivetrain(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        inp = new InputManager();
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.a, () -> servoPos += 0.05));
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.b, () -> servoPos -= 0.05));
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.x, () -> {
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }));
        inp.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> Math.abs(TARGET - left.getCurrentPosition()) < 10,
                () -> {
                    gamepad1.rumble(500);
                    gamepad1.setLedColor(0, 255, 0, 500);
                })
        );
    }

    double servoPos = 0;

    @Override
    public void loop() {
        inp.update();
        dt.setPowers(dt.mix(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

        double motorPower = pid.update(TARGET, left.getCurrentPosition()) * 12 / voltageSensor.getVoltage();
        if (left.getCurrentPosition() < 800) {
            motorPower += HOLD_0;
        } else if (left.getCurrentPosition() < 1600) {
            motorPower += HOLD_1;
        } else {
            motorPower += HOLD_2;
        }
        motorPower *= 12 / voltageSensor.getVoltage();
        left.setPower(motorPower);
        right.setPower(motorPower);
        telemetry.addData("servo", servoPos);
        telemetry.addData("pid", motorPower);
        dash.addData("target", TARGET);
        dash.addData("position", left.getCurrentPosition());
        dash.addData("error", Math.abs(TARGET - left.getCurrentPosition()));
        dash.addData("errorAccum", pid.errorAccum);
        dash.addData("pid", motorPower);
        dash.update();
    }
}
