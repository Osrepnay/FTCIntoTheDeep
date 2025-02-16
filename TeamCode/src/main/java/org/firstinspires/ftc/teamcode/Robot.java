package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.Optional;

public class Robot {
    public enum State {
        INTAKE_SAMPLE(false, true),
        EXTEND(false, false),
        INTAKE_NOEXTEND(false, true),
        TRANSFER(false, false),
        PICKUP_SPECIMEN(false, false),
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
    private boolean transitionDone = false;

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
                if (input == Input.RIGHT_BUMPER) {
                    target = State.EXTEND;
                    transition = extendo.setIntake(Extendo.INTAKE_OFF)
                            .with(extendo.setWrist(Extendo.WRIST_RAISED));
                }
                break;
            case INTAKE_NOEXTEND:
                if (input == Input.RIGHT_BUMPER) {
                    target = State.TRANSFER;
                    transition = extendo.setIntake(Extendo.INTAKE_OFF)
                            .with(extendo.setWrist(Extendo.WRIST_TRANSFERRING))
                            .with(lift.setClaw(Lift.CLAW_OPENED))
                            .andThen(extendo.setIntake(Extendo.INTAKE_VOMIT))
                            .andThen(Task.delay(120))
                            .andThen(lift.setClaw(Lift.CLAW_CLOSED))
                            .andThen(Task.delay(200))
                            .andThen(extendo.setIntake(Extendo.INTAKE_OFF));
                }
                break;
            case EXTEND:
                if (input == Input.LEFT_BUMPER) {
                    target = State.INTAKE_SAMPLE;
                    transition = extendo.setIntake(Extendo.INTAKE_ON)
                            .andThen(extendo.setWrist(Extendo.WRIST_SAMPLE));
                } else if (input == Input.RIGHT_BUMPER) {
                    target = State.TRANSFER;
                    transition = extendo.setWrist(Extendo.WRIST_STAGE)
                            .with(extendo.retract())
                            .andThen(extendo.setWrist(Extendo.WRIST_TRANSFERRING))
                            .with(lift.setClaw(Lift.CLAW_OPENED))
                            .andThen(extendo.setIntake(Extendo.INTAKE_VOMIT))
                            .andThen(Task.delay(120))
                            .andThen(lift.setClaw(Lift.CLAW_CLOSED))
                            .andThen(Task.delay(200))
                            .andThen(extendo.setIntake(Extendo.INTAKE_OFF));
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
                } else if (input == Input.BUTTON_Y) {
                    target = State.INTAKE_NOEXTEND;
                    transition = extendo.setWrist(Extendo.WRIST_SAMPLE)
                            .andThen(extendo.setIntake(Extendo.INTAKE_ON));
                } else if (input == Input.BUTTON_X) {
                    target = State.PICKUP_SPECIMEN;
                    transition = lift.liftTo(Lift.MOTOR_SPECIMEN_PICKUP)
                            .with(lift.setWrist(Lift.WRIST_SPEC_PICKUP))
                            .with(lift.setClaw(Lift.CLAW_OPENED));
                }
                break;
            case PICKUP_SPECIMEN:
                if (input == Input.LEFT_BUMPER) {
                    target = State.TRANSFER;
                    transition = lift.setWrist(Lift.WRIST_TRANSFERRING)
                            .andThen(lift.setClaw(Lift.CLAW_CLOSED)
                                    .with(lift.retract()));
                } else if (input == Input.RIGHT_BUMPER) {
                    target = State.DUMP_SPECIMEN;
                    transition = lift.setClaw(Lift.CLAW_CLOSED)
                            .andThen(lift.liftTo(Lift.MOTOR_SPECIMEN_SCORE)
                                    .with(lift.setWrist(Lift.WRIST_SPEC_SCORE)));

                }
                break;
            case DUMP_SPECIMEN:
                if (input == Input.LEFT_BUMPER) {
                    target = State.TRANSFER;
                    transition = lift.setClaw(Lift.CLAW_OPENED)
                            .andThen(lift.setWrist(Lift.WRIST_TRANSFERRING))
                            .andThen(lift.retract()
                                    .with(lift.setClaw(Lift.CLAW_CLOSED)));
                }
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
                /*
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
                 */
        }
        State finalTarget = target;
        return Optional.ofNullable(transition)
                .map(t -> new Task()
                        .oneshot(() -> {
                            state = finalTarget;
                            transitionDone = false;
                        })
                        .andThen(t)
                        .andThen(new Task().oneshot(() -> transitionDone = true))
                        .resources(this));
    }

    public Task transitionTask(Input input) {
        return Task.defer(() -> getTransition(input).orElse(new Task())).resources(this);
    }

    public boolean isTransitionDone() {
        return transitionDone;
    }

}
