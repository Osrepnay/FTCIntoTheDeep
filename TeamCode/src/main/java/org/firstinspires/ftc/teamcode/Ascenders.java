package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Ascenders {
    public final DcMotor leftAscentMotor;
    public final DcMotor rightAscentMotor;

    public Ascenders(DcMotor leftAscentMotor, DcMotor rightAscentMotor) {
        this.leftAscentMotor = leftAscentMotor;
        this.rightAscentMotor = rightAscentMotor;
        this.leftAscentMotor.setDirection(DcMotor.Direction.REVERSE);
        this.rightAscentMotor.setDirection(DcMotor.Direction.REVERSE);

        this.leftAscentMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightAscentMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Ascenders(HardwareMap hardwareMap) {
        this(
                hardwareMap.get(DcMotor.class, "leftAscentMotor"),
                hardwareMap.get(DcMotor.class, "rightAscentMotor")
        );
    }

    public void setPower(double power) {
        leftAscentMotor.setPower(power);
        rightAscentMotor.setPower(power);
    }
}
