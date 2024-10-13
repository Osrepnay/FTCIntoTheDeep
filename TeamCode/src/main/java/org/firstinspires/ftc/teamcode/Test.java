package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

@TeleOp
public class Test extends OpMode {
    // nw, ne, sw, se
    private DcMotor[] wheels = new DcMotor[4];

    private InputManager inputManager = new InputManager();
    private TaskRunner taskRunner = new TaskRunner();

    @Override
    public void init() {
        wheels = new DcMotor[]{
                hardwareMap.get(DcMotor.class, "wheelFrontLeft"),
                hardwareMap.get(DcMotor.class, "wheelFrontRight"),
                hardwareMap.get(DcMotor.class, "wheelBackLeft"),
                hardwareMap.get(DcMotor.class, "wheelBackRight")
        };
        for (DcMotor wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        wheels[0].setDirection(DcMotor.Direction.REVERSE);
        wheels[2].setDirection(DcMotor.Direction.REVERSE);
        extendo = hardwareMap.get(DcMotor.class, "extendo");
        lift = hardwareMap.get(DcMotor.class, "lift");
        extendoWrist = hardwareMap.get(Servo.class, "extendoWrist");
        extendoClaw = hardwareMap.get(Servo.class, "extendoClaw");
        liftWrist = hardwareMap.get(Servo.class, "liftWrist");
        liftClaw = hardwareMap.get(Servo.class, "liftClaw");

        // inputManager.addTrigger(new Trigger());
    }

    @Override
    public void loop() {
        inputManager.update();
        taskRunner.update();

        double moveX = gamepad1.right_stick_x;
        double moveY = -gamepad1.right_stick_y;
        double rotate = gamepad1.left_stick_x;
        wheels[0].setPower(moveY + moveX + rotate);
        wheels[1].setPower(moveY - moveX - rotate);
        wheels[2].setPower(moveY - moveX + rotate);
        wheels[3].setPower(moveY + moveX - rotate);

        double
    }
}
