package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
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

import java.util.List;

@Config
@TeleOp
public class LiftTest extends OpMode {
    DcMotor left;
    DcMotor right;
    InputManager inp;
    Drivetrain dt;
    VoltageSensor voltageSensor;
    Telemetry dash;

    public static PIDController pid = new PIDController(0.008, 0.00005, 0.4, 20000, 0.8, 100);
    public static long TARGET = 0;
    public static double HOLD_MAX = 0.07;
    public static long delayMs = 0;

    private List<LynxModule> hubs;

    @Override
    public void init() {
        pid.reset();
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
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    double servoPos = 0;
    long last = 0;

    @Override
    public void loop() {
        hubs.forEach(LynxModule::clearBulkCache);

        inp.update();
        dt.setPowers(dt.mix(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

        double motorPower = pid.update(TARGET, left.getCurrentPosition()) * 14 / voltageSensor.getVoltage();
        motorPower += HOLD_MAX * TARGET / 2400;
        left.setPower(motorPower);
        right.setPower(motorPower);
        telemetry.addData("servo", servoPos);
        telemetry.addData("pid", motorPower);
        dash.addData("target", TARGET);
        dash.addData("position", left.getCurrentPosition());
        dash.addData("error", Math.abs(TARGET - left.getCurrentPosition()));
        dash.addData("errorAccum", pid.getErrorAccum());
        dash.addData("pid", motorPower);
        long time = System.currentTimeMillis();
        dash.addData("hz", 1000.0 / (time - last));
        last = time;
        dash.update();
        try {
            Thread.sleep(delayMs);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
