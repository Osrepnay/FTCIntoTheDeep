package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

@TeleOp
public class Test extends OpMode {
    private DcMotorEx[] wheels;
    private Robot robot;
    private final InputManager inputManager = new InputManager();
    private final TaskRunner taskRunner = new TaskRunner();

    private boolean isLift = false;

    @Override
    public void init() {
        wheels = new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "wheelFrontLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelFrontRight"),
                hardwareMap.get(DcMotorEx.class, "wheelBackLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelBackRight")
        };
        for (DcMotor wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        wheels[0].setDirection(DcMotor.Direction.REVERSE);
        wheels[2].setDirection(DcMotor.Direction.REVERSE);

        robot = new Robot(hardwareMap);

        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.left_bumper,
                () -> isLift = !isLift
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.x,
                () -> taskRunner.sendTask(isLift ? robot.lift.toTransfer() : robot.extendo.toTransfer())
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.y,
                () -> taskRunner.sendTask(isLift ? robot.lift.toLift() : robot.extendo.toExtend())
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.b,
                () -> taskRunner.sendTask(isLift ? robot.lift.dump() : robot.extendo.toRummage())
        ));
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

        if (isLift) {
            robot.lift.setMotorPos(robot.lift.getMotorPos() + Math.round(gamepad1.left_trigger * 3));
        } else {
            robot.extendo.setMotorPos(robot.extendo.getMotorPos() + Math.round(gamepad1.right_trigger * 3));
        }
    }
}
