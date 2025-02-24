package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TrajectoryTaskBuilder.actionToTask;
import static org.firstinspires.ftc.teamcode.TrajectoryTaskBuilder.taskToAction;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

@Autonomous
public class Bongo extends OpMode {
    Robot robot;
    TaskRunner taskRunner;
    MecanumDrive drive;
    InputManager inp;

    private final double TILE = 70.0 / 3;

    private static Pose2d forward(Pose2d pose, double dist) {
        double dx = dist * Math.cos(pose.heading.toDouble());
        double dy = dist * Math.sin(pose.heading.toDouble());
        return new Pose2d(new Vector2d(pose.position.x + dx, pose.position.y + dy), pose.heading);
    }

    private static Rotation2d flip(Rotation2d ang) {
        return ang.plus(Math.toRadians(180));
    }

    private boolean checkSample = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        taskRunner = new TaskRunner();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0.5 * TILE + 4.25, -2.5 * TILE - 3.5, Math.toRadians(-90)));
        inp = new InputManager();
        // stalling < loop times
        /*
        inp.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> (robot.getState().INTAKING || checkSample) && robot.extendo.hasSample(),
                () -> robot.extendo.setIntake(Extendo.INTAKE_OFF)
        ));
         */
        Pose2d chamber = new Pose2d(0.5 * TILE - 2, -1.5 * TILE - 0.3, Math.toRadians(-90));
        Vector2d adjust = new Vector2d(-1.8, 1.4);
        Pose2d[] chambers = Stream.iterate(chamber, p -> new Pose2d(p.position.plus(adjust), p.heading))
                .limit(5)
                .toArray(Pose2d[]::new);
        Pose2d[] chamberSpin = Arrays.stream(chambers)
                .map(p -> p.plus(new Twist2d(new Vector2d(10, 0), Math.toRadians(140))))
                .toArray(Pose2d[]::new);
        Pose2d[] chamberShove = Arrays.stream(chambers)
                .map(p -> p.plus(new Twist2d(new Vector2d(0, -3), Math.toRadians(0))))
                .toArray(Pose2d[]::new);
        Pose2d pickup = new Pose2d(1.5 * TILE + 3, -2.5 * TILE + 0, Math.toRadians(89.99));
        Vector2d pAdjust = new Vector2d(0, -0.3);
        Pose2d[] pickups = Stream.iterate(pickup, p -> new Pose2d(p.position.plus(pAdjust), p.heading))
                .limit(6)
                .toArray(Pose2d[]::new);
        Pose2d lineup = forward(pickup, 3);
        Pose2d moveFirst = new Pose2d(0.5 * TILE + 4.25 + 9.5, -2.5 * TILE - 3.5 + 18, Math.toRadians(36));
        Pose2d firstPushed = forward(moveFirst, 4.5);
        Pose2d moveSecond = new Pose2d(0.5 * TILE + 4.25 + 21.4, -2.5 * TILE - 3.5 + 19.5, Math.toRadians(36));
        Pose2d secondPushed = forward(moveSecond, 4);
        Pose2d moveThird = new Pose2d(0.5 * TILE + 4.25 + 29.1, -2.5 * TILE - 3.5 + 20.9, Math.toRadians(31));
        Pose2d thirdPushed = forward(moveThird, 4);
        Pose2d thirdDump = new Pose2d(0.5 * TILE + 4.25 + 22.2, -2.5 * TILE - 3.5 + 22, Math.toRadians(-51));
        VelConstraint lineupSlowdown = new AngularVelConstraint(Math.PI * 0.8);
        AccelConstraint lineupSlowdownAccel = new ProfileAccelConstraint(-25, 40);
        VelConstraint firstPushConstraint = new TranslationalVelConstraint(5);
        VelConstraint pushConstraint = new TranslationalVelConstraint(5);
        TurnConstraints raah = new TurnConstraints(
                Math.PI * 1.2,
                -Math.PI * 4,
                Math.PI * 4
        );
        Task waitRobotTask = new Task().update(() -> robot.isTransitionDone());
        taskRunner.sendTask(robot.init());
        taskRunner.sendTask(actionToTask(drive.actionBuilder(drive.pose)
                // score first
                .afterTime(0, taskToAction(robot.lift.liftTo(Lift.MOTOR_SPECIMEN_SCORE)
                        .with(robot.lift.setWrist(Lift.WRIST_SPEC_SCORE))
                        .with(new Task().oneshot(() -> robot.setState(Robot.State.DUMP_SPECIMEN)))))
                .waitSeconds(0.4)
                .setReversed(true).splineTo(chambers[0].position, flip(chambers[0].heading))
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))

                // pickup and score second
                .afterTime(0.5, taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))
                .endTrajectory()
                .strafeToSplineHeading(chamberSpin[0].position, chamberSpin[0].heading)
                .splineToSplineHeading(lineup, flip(lineup.heading))
                .strafeTo(pickups[0].position)
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.lift.setClaw(Lift.CLAW_CLOSED))))
                .afterTime(0, taskToAction(robot.transitionTask(Robot.Input.RIGHT_BUMPER)))
                .waitSeconds(0.6)
                .strafeToSplineHeading(chambers[1].position, chambers[1].heading)
                .endTrajectory()
                .strafeToConstantHeading(chamberShove[1].position)
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))

                // sweep first
                .afterTime(0.5, taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))
                .afterTime(0.1, taskToAction(robot.extendo.extend()
                        .with(robot.extendo.setWrist(Extendo.WRIST_SAMPLE))
                        .andThen(robot.extendo.setIntake(Extendo.INTAKE_ON))))
                .endTrajectory().splineTo(moveFirst.position, moveFirst.heading, lineupSlowdown, lineupSlowdownAccel)
                .splineTo(firstPushed.position, firstPushed.heading, firstPushConstraint)
                .waitSeconds(0.2)
                .afterTime(0.2, taskToAction(robot.extendo.setIntake(Extendo.INTAKE_OFF)))
                .turn(Math.toRadians(-95), raah)
                .stopAndAdd(taskToAction(robot.extendo.spew()))

                // sweep second
                .afterTime(0.2, taskToAction(robot.extendo.setIntake(Extendo.INTAKE_ON)))
                .endTrajectory().splineToSplineHeading(
                        moveSecond,
                        moveSecond.heading.plus(Math.toRadians(-30)),
                        lineupSlowdown,
                        lineupSlowdownAccel)
                .splineToConstantHeading(secondPushed.position, secondPushed.heading, pushConstraint)
                .waitSeconds(0.2)
                .afterTime(0.2, taskToAction(robot.extendo.setIntake(Extendo.INTAKE_OFF)))
                .turn(Math.toRadians(-105), raah)
                .stopAndAdd(taskToAction(robot.extendo.spew()))
                .afterTime(0, taskToAction(robot.extendo.setWrist(Extendo.WRIST_TRANSFERRING)
                        .with(robot.extendo.retract())))

                /*
                // sweep third
                .afterTime(0.2, taskToAction(robot.extendo.setIntake(Extendo.INTAKE_ON)))
                .endTrajectory().strafeToLinearHeading(moveThird.position, moveThird.heading, lineupSlowdown, lineupSlowdownAccel)
                .strafeToConstantHeading(thirdPushed.position, pushConstraint)
                .waitSeconds(0.2)
                .afterTime(0.2, taskToAction(robot.extendo.setIntake(Extendo.INTAKE_OFF)))
                .endTrajectory().strafeToSplineHeading(thirdDump.position, thirdDump.heading)
                .stopAndAdd(taskToAction(robot.extendo.spew()))
                .afterTime(0, taskToAction(robot.extendo.setWrist(Extendo.WRIST_TRANSFERRING)
                        .with(robot.extendo.retract())))
                 */

                // pickup and score third
                .endTrajectory()
                .splineToSplineHeading(lineup, flip(lineup.heading))
                .strafeTo(pickups[2].position)
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.lift.setClaw(Lift.CLAW_CLOSED))))
                .afterTime(0, taskToAction(robot.transitionTask(Robot.Input.RIGHT_BUMPER)))
                .waitSeconds(0.6)
                .strafeToSplineHeading(chambers[2].position, chambers[2].heading)
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))

                // pickup and score fourth
                .endTrajectory()
                .afterDisp(0.5, taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))
                .strafeToSplineHeading(chamberSpin[2].position, chamberSpin[2].heading)
                .splineToSplineHeading(lineup, flip(lineup.heading))
                .strafeTo(pickups[3].position)
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.lift.setClaw(Lift.CLAW_CLOSED))))
                .afterTime(0, taskToAction(robot.transitionTask(Robot.Input.RIGHT_BUMPER)))
                .waitSeconds(0.6)
                .strafeToSplineHeading(chambers[3].position, chambers[3].heading)
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))

                .build()
        ).resources(robot));
    }

    private long last = 0;

    @Override
    public void loop() {
        telemetry.addData("state", robot.getState());
        telemetry.addData("imu", drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
        long now = System.currentTimeMillis();
        telemetry.addData("hz", 1000.0 / (now - last));
        last = now;

        taskRunner.update();
        robot.update();
    }
}
