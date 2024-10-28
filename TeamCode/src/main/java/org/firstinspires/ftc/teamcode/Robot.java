package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public final Extendo extendo;
    public final Lift lift;

    public Robot(Extendo extendo, Lift lift) {
        this.extendo = extendo;
        this.lift = lift;
    }

    public Robot(HardwareMap hardwareMap) {
        this.extendo = new Extendo(hardwareMap);
        this.lift = new Lift(hardwareMap);
    }
}
