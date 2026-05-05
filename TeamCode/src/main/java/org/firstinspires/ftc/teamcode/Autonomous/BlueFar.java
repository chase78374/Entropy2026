package org.firstinspires.ftc.teamcode.Autonomous;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.flywheel;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class BlueFar extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    public IntakeSubsystem intake;
    private int pathState; // Current autonomous path state (state machine)private Path scorePreload;
    private Path scorePreload;
    private Servo l = null;
    private Servo r = null;
    private flywheel shooter;
    private boolean shooterActive = false;

    private PathChain scoreThePreload, grabHumanZone1, grabHumanZone2, scorePickup1, grabSpike, scoreSpike, grabField1, scoreField1, grabField2, scoreField2, park;

    private Timer pathTimer, opmodeTimer, actionTimer;


    private final Pose startPose = new Pose(85, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(85, 15, Math.toRadians(68.5));
    private final Pose humanZone1 = new Pose(132, 13.5, Math.toRadians(0));
    private final Pose bezierCurve1 = new Pose(116.500, 8.250);
    private final Pose bezierCurve2 = new Pose(80.500, 36.000);
    private final Pose bezierCurve3 = new Pose(102.500, 36.000);
    private final Pose bezierCurve4 = new Pose(132.000, 20.000);
    private final Pose humanZone2 = new Pose(132, 8, Math.toRadians(0));
    private final Pose grabSpike3 = new Pose(130, 35, Math.toRadians(0));
    private final Pose grabFieldBall1 = new Pose(130, 40, Math.toRadians(0));
    private final Pose grabFieldBall2 = new Pose(132, 47.5, Math.toRadians(90));
    private final Pose parkingPose = new Pose(85, 25, Math.toRadians(0));


    private int shootStep = 0;


    private Timer stateGapTimer;
    private boolean waitingForGap = false;
    private int nextQueuedState = -1;

    private double[] gapAfterState = new double[4000];


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        stateGapTimer = new Timer();
        stateGapTimer.resetTimer();
        shooter = new flywheel(
                hardwareMap.get(DcMotorEx.class, "shooter"),
                hardwareMap.get(DcMotorEx.class, "shooter2")

        );

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("Timer conversion complete");
        telemetry.update();
        l = (Servo) hardwareMap.get(Servo.class, "l");
        r = (Servo) hardwareMap.get(Servo.class, "r");
        l.setDirection(Servo.Direction.FORWARD);
        r.setDirection(Servo.Direction.REVERSE);
    }

    private void transitionTo(int newState) {
        double gap = 0;

        if (pathState >= 0 && pathState < gapAfterState.length) {
            gap = gapAfterState[pathState];
        }

        if (gap > 0) {
            waitingForGap = true;
            nextQueuedState = newState;
            stateGapTimer.resetTimer();
        } else {
            setPathState(newState);
        }
    }


    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setBrakingStrength(1);


        grabHumanZone1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, humanZone1))
                .setTangentHeadingInterpolation()
                .build();
        grabHumanZone2 = follower.pathBuilder()
                .addPath(new BezierCurve(humanZone1, bezierCurve1, humanZone2))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(humanZone2, scorePose))
                .setLinearHeadingInterpolation(humanZone2.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration(.7)
                .build();

        grabSpike = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, bezierCurve2, grabSpike3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabSpike3.getHeading())
                .setGlobalDeceleration(.7)
                .build();

        scoreSpike = follower.pathBuilder()
                .addPath(new BezierLine(grabSpike3, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        grabField1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, bezierCurve3, grabFieldBall1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabFieldBall1.getHeading())
                .build();


        scoreField1 = follower.pathBuilder()
                .addPath(new BezierLine(grabFieldBall1, scorePose))
                .setLinearHeadingInterpolation(grabFieldBall1.getHeading(), scorePose.getHeading(), .65)
                .setGlobalDeceleration(.7)
                .build();

        grabField2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, bezierCurve4, grabFieldBall2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabFieldBall2.getHeading(), .65)
                .setGlobalDeceleration(.7)
                .build();


        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkingPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading())
                .build();

    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();

        double shooterCloseTarget = 1700;
        //shooter.updateShooterPIDF(shooterCloseTarget);
        // shooter.hood.setPosition(0.4);
        //shooter.blocker.setPosition(1);
        follower.followPath(scorePreload);
        shooter.bangBang(1650);
        l.setPosition(1);
        r.setPosition(1);
        setPathState(67);

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        shootStep = 0;
    }

    public void autonomousPathUpdate() {
        if (waitingForGap) {
            if (stateGapTimer.getElapsedTimeSeconds() >= gapAfterState[pathState]) {
                waitingForGap = false;
                setPathState(nextQueuedState);
            }
            return;
        }
        switch (pathState) {
            case 67:
                if (!follower.isBusy()) {
                    intake.intake("in");
                    shooter.bangBang(1650);
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    intake.intake("not");
                    shooter.bangBang(0);
                    l.setPosition(.3);
                    r.setPosition(.5);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(grabHumanZone1);
                    intake.intake("in");
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabHumanZone2);
                    intake.intake("in");
                    shooter.bangBang(1650);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    intake.intake("not");
                    shooter.bangBang(1650);
                    l.setPosition(1);
                    r.setPosition(1);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    intake.intake("in");
                    shooter.bangBang(1650);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    intake.intake("in");
                    shooter.bangBang(0);
                    l.setPosition(.3);
                    r.setPosition(.5);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(grabSpike);
                    intake.intake("in");
                    shooter.bangBang(1650);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    follower.followPath(scoreSpike);
                    intake.intake("not");
                    shooter.bangBang(1650);
                    l.setPosition(1);
                    r.setPosition(1);
                    setPathState(9);
                }
                break;
            case 9:
                    if (!follower.isBusy()) {
                        intake.intake("in");
                        shooter.bangBang(1650);
                        setPathState(10);
                    }
                    break;
            case 10:
                if (!follower.isBusy()){
                    follower.followPath(grabField1);
                    intake.intake("in");
                    shooter.bangBang(1650);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()){
                    follower.followPath(scoreField1);
                    intake.intake("not");
                    shooter.bangBang(1650);
                    l.setPosition(1);
                    r.setPosition(1);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()){
                    follower.followPath(grabField2);
                    intake.intake("in");
                    shooter.bangBang(1650);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()){
                    follower.followPath(scoreField2);
                    intake.intake("not");
                    shooter.bangBang(1650);
                    l.setPosition(1);
                    r.setPosition(1);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()){
                    intake.intake("in");
                    shooter.bangBang(1650);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()){
                    follower.followPath(park);
                    intake.intake("not");
                    shooter.bangBang(0);
                    l.setPosition(.3);
                    r.setPosition(.5);
                    setPathState(16);
                    setPathState(-1);
                }
                break;
            case -1:
                    break;





        }

    }
}