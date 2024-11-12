package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

@TeleOp
public class Test extends OpMode {
    private Drivetrain drivetrain;
    private Robot robot;
    private final InputManager inputManager = new InputManager();
    private final TaskRunner taskRunner = new TaskRunner();
    private Servo shit;

    private boolean panic = false;

    @Override
    public void init() {
        shit = hardwareMap.get(Servo.class, "liftWrist");
        drivetrain = new Drivetrain(hardwareMap);
        robot = new Robot(hardwareMap);

        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.left_bumper,
                () -> {
                    Robot.State prev = robot.getState().prev();
                    if (prev != null) {
                        robot.toState(prev).ifPresent(taskRunner::sendTask);
                    }
                }
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.right_bumper,
                () -> {
                    Robot.State next = robot.getState().next();
                    if (next != null) {
                        robot.toState(next).ifPresent(taskRunner::sendTask);
                    } else if (robot.getState() == Robot.State.LIFTING) {
                        taskRunner.sendTask(robot.dump());
                    }
                }
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.x,
                () -> robot.ascenders.setPower(1)
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.y,
                () -> robot.ascenders.setPower(-1)
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.END,
                () -> gamepad1.x || gamepad1.y,
                () -> robot.ascenders.setPower(0)
        ));

        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad2.left_bumper && gamepad2.right_bumper && !panic,
                () -> {
                    panic = true;
                    taskRunner.flush();
                }
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad2.left_bumper && gamepad2.right_bumper && panic,
                () -> {
                    panic = false;
                    taskRunner.sendTask(robot.init());
                }
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> panic && gamepad2.x,
                () -> robot.extendo.setWrist(Extendo.WRIST_TRANSFERRING)
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> panic && gamepad2.y,
                () -> robot.lift.setWrist(Lift.WRIST_TRANSFERRING)
        ));
    }

    boolean first = true;
    @Override
    public void loop() {
        if (first) {
            first = false;
            taskRunner.sendTask(robot.init());
        }

        drivetrain.setPowers(drivetrain.mix(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

        double triggerInput = gamepad1.right_trigger - gamepad1.left_trigger;
        if (!panic) {
            // TODO technically correct because of motor lock
            // very jank though
            robot.lift.setMotorInput(triggerInput);
            robot.extendo.setMotorInput(triggerInput);
        } else {
            robot.extendo.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);
            robot.lift.setMotorPower(-gamepad2.right_stick_y);
        }
        if (gamepad1.dpad_up) {
            shit.setPosition(0.1);
        }
        telemetry.addData("dump", gamepad1.dpad_up);

        telemetry.addData("state", robot.getState());
        telemetry.addData("panic", panic);
        telemetry.addData("extendo", robot.extendo.getMotorCurrentPosition());
        telemetry.addData("lift", robot.lift.getMotorCurrentPosition());
        telemetry.addData("left", gamepad2.left_bumper);
        telemetry.addData("right", gamepad2.right_bumper);

        inputManager.update();
        taskRunner.update();
    }
}
