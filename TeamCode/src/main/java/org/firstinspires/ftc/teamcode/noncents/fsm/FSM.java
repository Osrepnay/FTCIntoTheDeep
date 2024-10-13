package org.firstinspires.ftc.teamcode.noncents.fsm;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public interface FSM<A> {
    Task getTransition(A target);

    A getState();
}