package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.noncents.PIDController;
import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;

import java.util.List;

@Config
@TeleOp
public class Level4Recovery extends OpMode {
    DcMotor left;
    DcMotor right;
    InputManager inp;
    Drivetrain dt;
    Telemetry dash;

    @Override
    public void init() {
        dash = FtcDashboard.getInstance().getTelemetry();
        left = hardwareMap.get(DcMotor.class, "liftLeft");
        right = hardwareMap.get(DcMotor.class, "liftRight");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        dt = new Drivetrain(hardwareMap);
        inp = new InputManager();
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad2.x, () -> {
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            gamepad2.rumble(200);
        }));
    }

    double servoPos = 0;
    long last = 0;

    @Override
    public void loop() {
        inp.update();
        dt.setPowers(dt.mix(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x));

        double motorPower = gamepad2.right_trigger - gamepad2.left_trigger;
        left.setPower(motorPower);
        right.setPower(motorPower);
    }
}
