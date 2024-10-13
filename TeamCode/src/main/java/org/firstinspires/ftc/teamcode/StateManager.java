package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.noncents.fsm.FSM;
import org.firstinspires.ftc.teamcode.noncents.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public class StateManager implements FSM<StateManager.RobotState> {
    /*
          -----normal----
         |              |
    extending        scoring
         |
    searching
    */
    public enum RobotState {
        TRANSFER,
        EXTENDING,
        SEARCHING,
        SCORING
    }

    private RobotState state;

    private final Extendo extendo;
    private final Lift lift;

    public StateManager(HardwareMap hardwareMap) {
        extendo = new Extendo(
                hardwareMap.get(DcMotor.class, "extendo"),
                hardwareMap.get(Servo.class, "extendoWrist"),
                hardwareMap.get(Servo.class, "extendoClaw")
        );
        lift = new Lift(
                hardwareMap.get(DcMotor.class, "lift"),
                hardwareMap.get(Servo.class, "liftWrist"),
                hardwareMap.get(Servo.class, "liftClaw")
        );
    }

    @Override
    public Task getTransition(RobotState target) {
        Task task = null;
        DelayTask servoLong = new DelayTask(400);
        DelayTask servoShort = new DelayTask(200);
        switch (state) {
            case TRANSFER:
                if (target == RobotState.EXTENDING) {
                    task = new Task(() -> extendo.setWrist(Extendo.WristPosition.CLEARANCE))
                            .andThen(servoLong);
                } else if (target == RobotState.SCORING) {
                    task = new Task(() -> extendo.setClaw(Extendo.ClawPosition.OPEN))
                            .andThen(servoShort)
                            .andThen(new Task(() -> lift.setClaw(Lift.ClawPosition.CLOSED)))
                            .andThen(servoShort);
                }
                break;
            case EXTENDING:
                if (target == RobotState.TRANSFER) {
                    task = new Task(() -> extendo.setExtendoTargetPosition(0))
                            .andThen(new Task(() -> Math.abs(extendo.getExtendoTargetPosition()) < 5));
                } else if (target == RobotState.SEARCHING) {
                    task = new Task(() -> extendo.setClaw(Extendo.ClawPosition.OPEN))
                            .andThen(servoShort)
                            .andThen(new Task(() -> extendo.setWrist(Extendo.WristPosition.DOWN)))
                            .andThen(servoLong);
                }
                break;
            case SEARCHING:
                if (target == RobotState.EXTENDING) {
                    task = new Task(() -> extendo.setWrist(Extendo.WristPosition.CLEARANCE))
                            .andThen(servoLong)
                            .andThen(new Task(() -> extendo.setClaw(Extendo.ClawPosition.CLOSED)))
                            .andThen(servoShort);
                }
                break;
            case SCORING:
                if (target == RobotState.TRANSFER) {
                    task = new Task(() -> lift.setClaw(Lift.ClawPosition.OPEN))
                            .with(new Task(() -> lift.setWrist(Lift.WristPosition.TRANSFER)))
                            .andThen(servoLong)
                            .with(new Task(() -> lift.setLiftTargetPosition(0))
                                    .andThen(new Task(() -> Math.abs(lift.getLiftTargetPosition()) < 5)))
                            .andThen(new Task(() -> extendo.setClaw(Extendo.ClawPosition.CLOSED)))
                            .andThen(servoShort);
                }
                break;
        }
        state = target;
        return task == null ? null : task.with(new Task(() -> {}, "fsm"));
    }

    @Override
    public RobotState getState() {
        return state;
    }
}