package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.Optional;

public class Robot {
    public enum State {
        RUMMAGE,
        EXTENDING,
        TRANSFER,
        LIFTING;

        private static final State[] states = State.values();

        public State next() {
            if (this.ordinal() == states.length - 1) {
                return null;
            } else {
                return states[this.ordinal() + 1];
            }
        }

        public State prev() {
            if (this.ordinal() == 0) {
                return null;
            } else {
                return states[this.ordinal() - 1];
            }
        }
    }

    public final Extendo extendo;
    public final Lift lift;
    private State state;

    public Robot(Extendo extendo, Lift lift) {
        this.extendo = extendo;
        this.lift = lift;
    }

    public Robot(HardwareMap hardwareMap) {
        this.extendo = new Extendo(hardwareMap);
        this.lift = new Lift(hardwareMap);
    }

    public State getState() {
        return state;
    }

    // TODO this sucks....
    public Task dump() {
        return lift.setWrist(Lift.WRIST_DUMP)
                .andThen(lift.setClaw(Lift.CLAW_OPENED))
                .andThen(lift.setWrist(Lift.WRIST_UP))
                .andThen(lift.setClaw(Lift.CLAW_CLOSED));
    }

    public Optional<Task> toState(State toState) {
        Task transition = null;
        if (state == State.RUMMAGE && toState == State.EXTENDING) {
            transition = extendo.setClaw(Extendo.CLAW_CLOSED)
                    .andThen(extendo.setWrist(Extendo.WRIST_RAISED))
                    .resources(this);
        }
        if (state == State.EXTENDING && toState == State.RUMMAGE) {
            transition = extendo.setClaw(Extendo.CLAW_OPENED)
                    .andThen(extendo.setWrist(Extendo.WRIST_LOWERED))
                    .resources(this);
        }
        if (state == State.EXTENDING && toState == State.TRANSFER) {
            transition = extendo.setClaw(Extendo.CLAW_CLOSED)
                    .andThen(extendo.setWrist(Extendo.WRIST_TRANSFERRING))
                    .andThen(extendo.retract())
                    .resources(this);
        }
        if (state == State.TRANSFER && toState == State.EXTENDING) {
            transition = extendo.setClaw(Extendo.CLAW_CLOSED)
                    .andThen(extendo.setWrist(Extendo.WRIST_RAISED))
                    .andThen(extendo.unlock())
                    .resources(this);
        }
        if (state == State.TRANSFER && toState == State.LIFTING) {
            transition = extendo.setClaw(Extendo.CLAW_OPENED)
                    .with(lift.setClaw(Lift.CLAW_CLOSED))
                    .andThen(lift.setWrist(Lift.WRIST_UP))
                    .andThen(lift.unlock())
                    .resources(this);
        }
        if (state == State.LIFTING && toState == State.TRANSFER) {
            transition = lift.retract()
                    .andThen(lift.setWrist(Lift.WRIST_TRANSFERRING))
                    .andThen(extendo.setClaw(Extendo.CLAW_CLOSED)
                            .with(lift.setClaw(Lift.CLAW_OPENED)))
                    .resources(this);
        }

        if (transition == null) {
            return Optional.empty();
        } else {
            return Optional.of(new Task().oneshot(() -> state = toState).andThen(transition));
        }
    }
}
