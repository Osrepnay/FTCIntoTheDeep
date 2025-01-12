package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class Test extends OpMode {
    private Drivetrain drivetrain;
    private Robot robot;
    private List<LynxModule> hubs;
    private final InputManager inputManager = new InputManager();
    private final TaskRunner taskRunner = new TaskRunner();

    private boolean panic = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        robot = new Robot(hardwareMap);

        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.left_bumper,
                () -> taskRunner.sendTask(robot.transitionTask(Robot.Input.LEFT_BUMPER))
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.right_bumper,
                () -> taskRunner.sendTask(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.x,
                () -> taskRunner.sendTask(robot.transitionTask(Robot.Input.BUTTON_X))
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.y,
                () -> taskRunner.sendTask(robot.transitionTask(Robot.Input.BUTTON_Y))
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.a,
                () -> taskRunner.sendTask(robot.transitionTask(Robot.Input.BUTTON_A))
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad1.b,
                () -> taskRunner.sendTask(robot.transitionTask(Robot.Input.BUTTON_B))
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> robot.getState().INTAKING && robot.extendo.hasSample(),
                () -> taskRunner.sendTask(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> robot.getState() == Robot.State.EXTEND && gamepad1.options,
                () -> taskRunner.sendTask(robot.extendo.spew())
        ));
        inputManager.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> gamepad2.dpad_down,
                () -> {
                    taskRunner.sendTask(new Task()
                            .oneshot(() -> robot.lift.setMotorPower(-0.2))
                            .andThen(new Task().oneshot(() -> robot.lift.resetMotor()))
                    );
                }
        ));

        // bulk reads
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    boolean first = true;

    @Override
    public void loop() {
        if (first) {
            first = false;
            taskRunner.sendTask(robot.init());
        }

        hubs.forEach(LynxModule::clearBulkCache);

        telemetry.addData("y", -gamepad1.left_stick_y);
        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("rot", gamepad1.right_stick_x);
        double[] mixed = drivetrain.mix(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        drivetrain.setPowers(mixed);

        double triggerInput = gamepad1.right_trigger - gamepad1.left_trigger;
        if (!panic) {
            if (robot.getState().LIFT_UNLOCKED) {
                robot.lift.setMotorInput(triggerInput);
            }
        } else {
            robot.lift.setMotorPower(-gamepad2.right_stick_y);
        }

        telemetry.addData("color", Color.currentColor);
        telemetry.addData("state", robot.getState());
        telemetry.addData("panic", panic);
        telemetry.addData("lift", robot.lift.getMotorCurrentPosition());
        telemetry.addData("liftSetpoint", robot.lift.liftSetpoint);

        inputManager.update();
        taskRunner.update();
        robot.update();
    }
}
