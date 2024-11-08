package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

@TeleOp
public class Test extends OpMode {
    private Drivetrain drivetrain;
    private Robot robot;
    private final InputManager inputManager = new InputManager();
    private final TaskRunner taskRunner = new TaskRunner();

    @Override
    public void init() {
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
    }

    boolean first = true;
    @Override
    public void loop() {
        if (first) {
            first = false;
            taskRunner.sendTask(robot.init());
        }

        drivetrain.setPowers(drivetrain.mix(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

        // TODO technically correct because of motor lock
        // very jank though
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        robot.lift.setMotorPower(power);
        robot.extendo.setMotorPower(power);

        telemetry.addData("state", robot.getState());
        telemetry.addData("power", String.valueOf(power));
        telemetry.addData("extendo", String.valueOf(robot.extendo.getMotorCurrentPosition()));
        telemetry.addData("lift", String.valueOf(robot.lift.getMotorCurrentPosition()));

        inputManager.update();
        taskRunner.update();
    }
}
