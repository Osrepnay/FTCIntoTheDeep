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
    }

    @Override
    public void loop() {
        inputManager.update();
        taskRunner.update();

        drivetrain.setPowers(drivetrain.mix(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

        // TODO technically correct because of motor lock
        // very jank though
        robot.lift.setMotorPos(robot.lift.getMotorPos() + Math.round(gamepad1.left_trigger * 3));
        robot.extendo.setMotorPos(robot.extendo.getMotorPos() + Math.round(gamepad1.right_trigger * 3));
    }
}
