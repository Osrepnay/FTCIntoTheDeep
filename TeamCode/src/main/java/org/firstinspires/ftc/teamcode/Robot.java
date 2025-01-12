package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.Optional;

public class Robot {
    public enum State {
        INTAKE_SAMPLE(false, true),
        INTAKE_SPEC(false, true),
        EXTEND(false, false),
        TRANSFER(false, false),
        LIFT(true, false),
        DUMP_SAMPLE(true, false),
        DUMP_SAMPLE_WAIT(true, false),
        DUMP_SPECIMEN(true, false);

        public final boolean LIFT_UNLOCKED;
        public final boolean INTAKING;

        State(boolean liftControl, boolean intaking) {
            LIFT_UNLOCKED = liftControl;
            INTAKING = intaking;
        }
    }

    public enum Input {
        LEFT_BUMPER,
        RIGHT_BUMPER,
        BUTTON_X,
        BUTTON_Y,
        BUTTON_A,
        BUTTON_B,
    }

    public final Extendo extendo;
    public final Lift lift;
    private State state = State.TRANSFER;

    public Robot(Extendo extendo, Lift lift) {
        this.extendo = extendo;
        this.lift = lift;
    }

    public Robot(HardwareMap hardwareMap) {
        this.extendo = new Extendo(hardwareMap);
        this.lift = new Lift(hardwareMap);
    }

    public void update() {
        lift.update();
    }

    public Task init() {
        return extendo.setIntake(Extendo.INTAKE_OFF)
                .with(extendo.setWrist(Extendo.WRIST_TRANSFERRING))
                .with(extendo.retract())
                .with(lift.setWrist(Lift.WRIST_TRANSFERRING))
                .with(lift.setClaw(Lift.CLAW_CLOSED))
                .andThen(lift.retract())
                .resources(this);
    }

    public State getState() {
        return state;
    }

    private Optional<Task> getTransition(Input input) {
        State target = null;
        Task transition = null;
        switch (state) {
            case INTAKE_SAMPLE:
            case INTAKE_SPEC:
                if (input == Input.RIGHT_BUMPER) {
                    target = State.EXTEND;
                    transition = extendo.setIntake(Extendo.INTAKE_OFF)
                            .with(extendo.setWrist(Extendo.WRIST_RAISED));
                }
                break;
            case EXTEND:
                if (input == Input.LEFT_BUMPER) {
                    target = State.INTAKE_SAMPLE;
                    transition = extendo.setIntake(Extendo.INTAKE_ON)
                            .andThen(extendo.setWrist(Extendo.WRIST_SAMPLE));
                } else if (input == Input.BUTTON_X) {
                    target = State.INTAKE_SPEC;
                    transition = extendo.setIntake(Extendo.INTAKE_ON)
                            .andThen(extendo.setWrist(Extendo.WRIST_SPEC));
                } else if (input == Input.RIGHT_BUMPER) {
                    target = State.TRANSFER;
                    transition = extendo.retract()
                            .andThen(extendo.setWrist(Extendo.WRIST_TRANSFERRING)
                                    .with(lift.setClaw(Lift.CLAW_OPENED))
                                    .andThen(extendo.setIntake(Extendo.INTAKE_VOMIT))
                                    .andThen(Task.delay(200))
                                    .andThen(lift.setClaw(Lift.CLAW_CLOSED))
                                    .andThen(Task.delay(100))
                                    .andThen(extendo.setIntake(Extendo.INTAKE_OFF))
                            );
                }
                break;
            case TRANSFER:
                if (input == Input.LEFT_BUMPER) {
                    target = State.EXTEND;
                    transition = extendo.setWrist(Extendo.WRIST_RAISED)
                            .with(extendo.extend());
                } else if (input == Input.RIGHT_BUMPER) {
                    target = State.DUMP_SAMPLE;
                    transition = lift.liftTo(Lift.MOTOR_MAX)
                            .with(lift.setWrist(Lift.WRIST_LIFT))
                            .andThen(lift.setWrist(Lift.WRIST_DUMP));
                } else if (input == Input.BUTTON_X) {
                    target = State.DUMP_SPECIMEN;
                    transition = lift.liftTo(Lift.MOTOR_CHAMBER_SCORE_SETUP)
                            .andThen(lift.setWrist(Lift.WRIST_SPEC));
                }
                break;
            case LIFT:
                // TODO dead code
                /*
                if (input == Input.LEFT_BUMPER) {
                    target = State.TRANSFER;
                    transition = lift.retract()
                            .with(lift.setWrist(Lift.WRIST_TRANSFERRING));
                } else if (input == Input.BUTTON_X) {
                    target = State.DUMP_SPECIMEN;
                    transition = lift.setWrist(Lift.WRIST_SPEC);
                } else if (input == Input.RIGHT_BUMPER) {
                    target = State.DUMP_SAMPLE;
                    transition = lift.setWrist(Lift.WRIST_DUMP)
                            .andThen(lift.setClaw(Lift.CLAW_OPENED))
                            .andThen(Task.delay(200))
                            .andThen(transitionTask(Input.DUMPING_SAMPLE_CLAW_DONE));
                }
                 */
                break;
            case DUMP_SAMPLE:
                if (input == Input.LEFT_BUMPER) {
                    target = State.TRANSFER;
                    transition = lift.setWrist(Lift.WRIST_LIFT)
                            .andThen(lift.retract().with(lift.setWrist(Lift.WRIST_TRANSFERRING)));
                } else if (input == Input.RIGHT_BUMPER) {
                    target = State.DUMP_SAMPLE_WAIT;
                    transition = lift.setClaw(Lift.CLAW_OPENED);
                }
                break;
            case DUMP_SAMPLE_WAIT:
                if (input == Input.LEFT_BUMPER) {
                    target = State.TRANSFER;
                    transition = lift.setWrist(Lift.WRIST_LIFT)
                            .with(lift.setClaw(Lift.CLAW_CLOSED))
                            .andThen(lift.retract()).with(lift.setWrist(Lift.WRIST_TRANSFERRING));
                }
                break;
            case DUMP_SPECIMEN:
                if (input == Input.LEFT_BUMPER) {
                    target = State.TRANSFER;
                    transition = lift.setWrist(Lift.WRIST_TRANSFERRING)
                            .with(lift.retract());
                } else if (input == Input.RIGHT_BUMPER) {
                    target = State.TRANSFER;
                    transition = new Task().oneshot(() -> lift.setMotorPower(Lift.MOTOR_CLIP_POWER))
                            .with(Task.delay(550))
                            .andThen(lift.setClaw(Lift.CLAW_OPENED))
                            .andThen(lift.setWrist(Lift.WRIST_TRANSFERRING)
                                    .andThen(lift.setClaw(Lift.CLAW_CLOSED))
                                    .with(lift.retract()));
                }
                break;
        }
        State finalTarget = target;
        return Optional.ofNullable(transition)
                .map(t -> new Task().oneshot(() -> state = finalTarget).andThen(t).resources(this));
    }

    public Task transitionTask(Input input) {
        return Task.defer(() -> getTransition(input).orElse(new Task())).resources(this);
    }

}
