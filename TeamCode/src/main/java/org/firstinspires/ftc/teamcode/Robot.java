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
    public final Ascenders ascenders;
    private State state = State.TRANSFER;

    public Robot(Extendo extendo, Lift lift, Ascenders ascenders) {
        this.extendo = extendo;
        this.lift = lift;
        this.ascenders = ascenders;
    }

    public Robot(HardwareMap hardwareMap) {
        this.extendo = new Extendo(hardwareMap);
        this.lift = new Lift(hardwareMap);
        this.ascenders = new Ascenders(hardwareMap);
    }

    public Task init() {
        return extendo.retract()
                .andThen(lift.retract())
                .andThen(extendo.setClaw(Extendo.CLAW_CLOSED)
                    .with(extendo.setWrist(Extendo.WRIST_TRANSFERRING))
                    .andThen(lift.setWrist(Lift.WRIST_TRANSFERRING)
                            .with(lift.setClaw(Lift.CLAW_OPENED))))
                .resources(this);
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
                    .andThen(extendo.setWrist(Extendo.WRIST_RAISED))
                    .andThen(extendo.retract())
                    .andThen(extendo.setWrist(Extendo.WRIST_TRANSFERRING))
                    .resources(this);
        }
        if (state == State.TRANSFER && toState == State.EXTENDING) {
            transition = extendo.setClaw(Extendo.CLAW_CLOSED)
                    .andThen(extendo.setWrist(Extendo.WRIST_RAISED))
                    .andThen(extendo.unlock())
                    .resources(this);
        }
        if (state == State.TRANSFER && toState == State.LIFTING) {
            transition = lift.setClaw(Lift.CLAW_CLOSED)
                    .with(Task.delay(200).andThen(extendo.setClaw(Extendo.CLAW_OPENED)))
                    .andThen(lift.setWrist(Lift.WRIST_UP))
                    .andThen(lift.unlock())
                    .resources(this);
        }
        if (state == State.LIFTING && toState == State.TRANSFER) {
            if (lift.getMotorCurrentPosition() < 1000) {
                transition = lift.retract()
                        .andThen(lift.setWrist(Lift.WRIST_TRANSFERRING))
                        .andThen(extendo.setClaw(Extendo.CLAW_CLOSED))
                        .andThen(lift.setClaw(Lift.CLAW_OPENED))
                        .resources(this);
            }
        }

        if (transition == null) {
            return Optional.empty();
        } else {
            return Optional.of(new Task().oneshot(() -> state = toState).andThen(transition));
        }
    }
}
