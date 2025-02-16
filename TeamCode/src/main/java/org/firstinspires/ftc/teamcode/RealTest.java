package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;

@Config
@TeleOp
public class RealTest extends OpMode {
    Servo servo;
    CRServo crServo;
    DcMotor motor;
    InputManager inp;
    Drivetrain dt;
    public static String motorName = "";
    String lastMotorName = motorName;
    public static String servoName = "extendoPivot";
    String lastServoName = servoName;
    public static String crServoName = "";
    String lastCrServoName = crServoName;

    @Override
    public void init() {
        if (!crServoName.isEmpty()) {
            crServo = hardwareMap.get(CRServo.class, crServoName);
        }
        if (!servoName.isEmpty()) {
            servo = hardwareMap.get(Servo.class, servoName);
        }
        if (!motorName.isEmpty()) {
            motor = hardwareMap.get(DcMotor.class, motorName);
        }
        dt = new Drivetrain(hardwareMap);
        inp = new InputManager();
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.a, () -> servoPos += 0.005));
        inp.addTrigger(new Trigger(Trigger.TriggerType.BEGIN, () -> gamepad1.b, () -> servoPos -= 0.005));
    }

    public static double servoPos = 0;
    public static double crServoPos = 0;

    @Override
    public void loop() {
        inp.update();
        dt.setPowers(dt.mix(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
        if (!motorName.equals(lastMotorName)) {
            lastMotorName = motorName;
            motor = hardwareMap.get(DcMotor.class, motorName);
        }
        if (!servoName.equals(lastServoName)) {
            lastServoName = servoName;
            servo = hardwareMap.get(Servo.class, servoName);
            servoPos = 0;
        }
        if (!crServoName.equals(lastCrServoName)) {
            lastCrServoName = crServoName;
            crServo = hardwareMap.get(CRServo.class, crServoName);
            crServoPos = 0;
        }
        if (servo != null) {
            servo.setPosition(servoPos);
        }
        if (crServo != null) {
            crServo.setPower(crServoPos);
        }
        if (motor != null) {
            motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("motorpos", motor.getCurrentPosition());
        }
    }
}
