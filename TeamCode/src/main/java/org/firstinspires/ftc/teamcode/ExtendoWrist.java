package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.noncents.fsm.FSM;
import org.firstinspires.ftc.teamcode.noncents.tasks.ServoTask;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public class ExtendoWrist implements FSM<ExtendoWrist.ExtendoWristState> {
    public interface ExtendoWristState {
    }

    public enum NormalState implements ExtendoWristState {
        DOWN(0),
        CLEARANCE(0),
        UP(0);
        public final double SERVO_POS;

        NormalState(double servoPos) {
            SERVO_POS = servoPos;
        }
    }

    public class TransitionState implements ExtendoWristState {
        public final NormalState from;
        public final NormalState to;

        public TransitionState(NormalState from, NormalState normalState) {
            this.from = from;
            to = normalState;
        }
    }

    private ExtendoWristState currentState = NormalState.UP;
    private Servo extendoWrist;

    public ExtendoWrist(Servo extendoWrist) {
        this.extendoWrist = extendoWrist;
    }

    @Override
    public Task getTransition(ExtendoWristState target) {
        if (target == currentState) {
            return null;
        }
        if (currentState instanceof TransitionState)
        if (target instanceof NormalState) {
            NormalState targetNormal = (NormalState) target;
            return Task.composeSequential(
                    () -> {
                        currentState = new TransitionState(currentState, targetNormal);
                        return true;
                    },
                    new ServoTask(extendoWrist, targetNormal.SERVO_POS, 300, "extendoWrist"),
                    () -> {
                        currentState = target;
                        return true;
                    }
            );
        } else {
            return null;
        }
    }

    @Override
    public ExtendoWristState getState() {
        return currentState;
    }
}
