// Converted to Pedro Timer (already used). Replaced ElapsedTime with pedro Timer.
package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
//import org.firstinspires.ftc.teamcode. s.intake ;
//import org.firstinspires.ftc.teamcode. s.shooter ;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "blueclose", group = "Examples")
public class Bluesideig extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1,betwixtPickup1, betwixtPickup2, regurgitate1, scoreregurgitate1, betwixtPickup3, grabregurgitate2, scoreregurgitate2, parking, gaterelease, grabPickup2, scorePickup2;

    public double initialCloseChargeTime = 1;
    public double intakeCollectDuration   = .5;
    public double intakeCollectDurationReg = 1;
    public double shortPullDuration       = 0.1;
    public double intakeShootingInwards   = 2.0;
    public double closeShotMultiplier     = 0.0225;
    private static final double AUTO_SHOOT_RPM = 1700;
    public double preloadWait             = 1.2;


    private final Pose startPose = new Pose(30, 115, Math.toRadians(144));
    private final Pose scorePose = new Pose(53 , 87.5, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(30, 67.5, Math.toRadians(180));
    private final Pose pickup1BetwixtPose = new Pose(40, 67.5, Math.toRadians(160));
    private final Pose pickup2BetwixtPose = new Pose(48, 59, Math.toRadians(180));
    private final Pose pickup3BetwixtPose = new Pose(48, 34, Math.toRadians(180));
    private final Pose regurgitate1pose = new Pose(29, 70, Math.toRadians(150));
    private final Pose parkingPose = new Pose(46, 68, Math.toRadians(135));
    private final Pose pickup2Pose = new Pose(30, 90, Math.toRadians(180));
    private final Pose scorereg2Pose = new Pose(60, 84, Math.toRadians(180));


  //  private shooter  shooter;
    //private intake  intake;

    private int shootStep = 0;
    // =============================
// AUTO TURRET PID (AUTO ONLY)
// =============================

    private double autoTurretTargetTicks = 0.0;

    private double autoTurretIntegral = 0.0;
    private double autoTurretLastError = 0.0;

    private double autoTurretMaxPower = 0.6;

    // Copy your tuned turret values
    private double auto_kP = 0.01;
    private double auto_kI = 0.0;
    private double auto_kD = 0.0006;
    private double auto_kS = 0.05;

    private Timer autoTurretTimer;
    // =============================
// OPTIONAL STATE GAPS
// =============================

    private Timer stateGapTimer;
    private boolean waitingForGap = false;
    private int nextQueuedState = -1;

    // Set delay (seconds) AFTER each state number
// Example: gapAfterState[1] = 0.5; means wait 0.5s after state 1 before entering next
    private double[] gapAfterState = new double[4000];

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        autoTurretTimer = new Timer();
        autoTurretTimer.resetTimer();
        stateGapTimer = new Timer();
        stateGapTimer.resetTimer();



     //   shooter = new shooter (this);
       // intake = new intake (this);

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("Timer conversion complete");
        telemetry.update();
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

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setBrakingStrength(1);


        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1BetwixtPose.getHeading(), pickup1Pose.getHeading(), .6)
                .setGlobalDeceleration(.7)
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorereg2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(scorereg2Pose.getHeading(), pickup2Pose.getHeading())
                .setGlobalDeceleration(.7)
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration(.7)
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration(.7)
                .build();

        betwixtPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1BetwixtPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        betwixtPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2BetwixtPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), regurgitate1pose.getHeading())
                .build();


        scoreregurgitate1 = follower.pathBuilder()
                .addPath(new BezierLine(regurgitate1pose, scorePose))
                .setLinearHeadingInterpolation(regurgitate1pose.getHeading(), scorePose.getHeading(), .65)
                .setGlobalDeceleration(.7)
                .build();

        betwixtPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3BetwixtPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), regurgitate1pose.getHeading(),.65)
                .setGlobalDeceleration(.7)
                .build();


        parking = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,parkingPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading())
                .build();

        regurgitate1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, regurgitate1pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), regurgitate1pose.getHeading(), .65)
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
        setPathState(1);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

     //   updateAutoTurretPID();
      //  shooter.updateShooterPIDF(AUTO_SHOOT_RPM);
        telemetry.addData("state", pathState);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
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
            case 1:
                if (!follower.isBusy()) {
                    shootStep = 0;
                    actionTimer.resetTimer();
                    //shooter.updateShooterPIDF(1700);
                    autoTurretTargetTicks = 320;
                   // shooter.hood.setPosition(0.4);
                    //shooter.blocker.setPosition(1);
                    setPathState(2);
                }
                break;

            case 2:

                if (actionTimer.getElapsedTimeSeconds() >= initialCloseChargeTime) {
                    //shooter.updateShooterPIDF(1700);
                    autoTurretTargetTicks = 320;
                   // shooter.hood.setPosition(0.4);
                    //shooter.blocker.setPosition(1);
                    //intake.frontspin.setPower(1);
                    //intake.backspin1.setPower(1);
                    //intake.backspin2.setPower(-1);
                    follower.followPath(betwixtPickup1);
                    setPathState(3);
                }

                break;
//
            //           case 94:
//                if (actionTimer.getElapsedTimeSeconds() >= preloadWait) {
//                    //shooter.updateShooterPIDF(0);
//                    intake.frontspin.setPower(0);
            //                   intake.backspin.setPower(1);
//                   follower.followPath(betwixtPickup1);
//                    setPathState(3);
            //               }
            //              break;

            case 3:
                if (actionTimer.getElapsedTimeSeconds() >= preloadWait) {
                    //shooter.updateShooterPIDF(0);
                   // intake.frontspin.setPower(1);
                    //intake.backspin1.setPower(1);
                    //intake.backspin2.setPower(-1);
                    //shooter.blocker.setPosition(0);
                    follower.followPath(grabPickup1);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    //shooter.updateShooterPIDF(0);
                  //  intake.frontspin.setPower(0);
                   // intake.backspin1.setPower(0);
                    //intake.backspin2.setPower(0);
                    //intake.frontspin.setPower(1);
                    //intake.backspin1.setPower(1);
                    //intake.backspin2.setPower(-1);
                    actionTimer.resetTimer();
                    setPathState(41);
                }
                break;

            case 41:
                //shooter.updateShooterPIDF(0);
                if (actionTimer.getElapsedTimeSeconds() >= intakeCollectDuration+.5) {
                    //intake.frontspin.setPower(0);
                    //intake.backspin1.setPower(0);
                    //intake.backspin2.setPower(0);
                    //shooter.blocker.setPosition(1);
                    //shooter.updateShooterPIDF(1700);
                    follower.followPath(scorePickup1);
                    setPathState(5);
                }
                break;

            case 5:
                if (actionTimer.getElapsedTimeSeconds() >= initialCloseChargeTime+1.5) {
                    autoTurretTargetTicks = 320;
                  //  shooter.hood.setPosition(0.4);
                   // shooter.blocker.setPosition(1);
                   // intake.frontspin.setPower(1);
                    //intake.backspin1.setPower(1);
                    //intake.backspin2.setPower(-1);
                    setPathState(52);
                }
                break;


            case 52:
                if (actionTimer.getElapsedTimeSeconds() >= initialCloseChargeTime) {
                    //shooter.updateShooterPIDF(0);
                 //   shooter.hood.setPosition(0.4);
                  //  shooter.blocker.setPosition(0);
                   // intake.frontspin.setPower(1);
                   // intake.backspin1.setPower(1);
                   // intake.backspin2.setPower(-1);
                    autoTurretTargetTicks = 320;
                    follower.followPath(regurgitate1);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                 //   intake.frontspin.setPower(1);
                   // intake.backspin1.setPower(1);
                    //intake.backspin2.setPower(-1);
                  //  follower.followPath(regurgitate1);
                    follower.followPath(scoreregurgitate1);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    //shooter.updateShooterPIDF(0);
                    //shooter.blocker.setPosition(0);
                    follower.followPath(regurgitate1);
                    setPathState(89);
                }
                break;
            case 89:
                //shooter.updateShooterPIDF(0);
                if (actionTimer.getElapsedTimeSeconds() >= intakeCollectDurationReg+.5) {
                    //intake.frontspin.setPower(0);
                    //intake.backspin1.setPower(0);
                    //intake.backspin2.setPower(0);
                    //shooter.blocker.setPosition(1);
                    //shooter.hood.setPosition(.4);
                   // follower.followPath(scoreregurgitate1);
                    follower.followPath(scoreregurgitate1);
                    setPathState(8);
                }
                break;


            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= initialCloseChargeTime+1.1) {
                  //  shooter.blocker.setPosition(1);
                   // shooter.hood.setPosition(0.4);
                   // intake.frontspin.setPower(1);
                   // intake.backspin1.setPower(1);
                   // intake.backspin2.setPower(-1);
                    //shooter.updateShooterPIDF(1700);
                    follower.followPath(regurgitate1);
                    setPathState(101);
                }
                break;

            case 81:
                //shooter.updateShooterPIDF(1700);
                if (actionTimer.getElapsedTimeSeconds() >= initialCloseChargeTime) {
                 //   shooter.hood.setPosition(0.4);
                   // shooter.blocker.setPosition(0);
                   // intake.frontspin.setPower(1);
                   // intake.backspin1.setPower(1);
                   // intake.backspin2.setPower(-1);
                    autoTurretTargetTicks = 320;
                    setPathState(82);
                }
                break;

            case 82:
                //shooter.updateShooterPIDF(1700);
              //  shooter.hood.setPosition(0.4);
               // shooter.blocker.setPosition(0);
                autoTurretTargetTicks=320;
               // intake.frontspin.setPower(1);
               // intake.backspin1.setPower(1);
               // intake.backspin2.setPower(-1);
                actionTimer.resetTimer();
              //  follower.followPath(gaterelease);
                setPathState(9);

                break;

            case 9:
                if (actionTimer.getElapsedTimeSeconds() >= 1.2) {
                 //   intake.frontspin.setPower(1);
                  //  intake.backspin1.setPower(1);
                   // intake.backspin2.setPower(-1);
                    //shooter.blocker.setPosition(0);
                 //   follower.followPath(regurgitate1);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                   // shooter.blocker.setPosition(0);
                    //shooter.updateShooterPIDF(0);
                    setPathState(101);
                }
                break;

            case 101:
                //shooter.updateShooterPIDF(0);
                if (actionTimer.getElapsedTimeSeconds() >= intakeCollectDurationReg+1.2) {
                  //  intake.frontspin.setPower(0);
                   // intake.backspin1.setPower(0);
                    //intake.backspin2.setPower(0);
                    //shooter.blocker.setPosition(1);
                    //shooter.hood.setPosition(.4);
                    follower.followPath(scoreregurgitate1);
                    setPathState(67);
                }
                break;

            case 11:
                if (actionTimer.getElapsedTimeSeconds() >= initialCloseChargeTime+1.1) {
                  //  shooter.blocker.setPosition(1);
                   // shooter.hood.setPosition(0.4);
                   // intake.frontspin.setPower(1);
                  //  intake.backspin1.setPower(1);
                  //  intake.backspin2.setPower(-1);
                    setPathState(111);
                }
                break;

            case 111:
                //shooter.updateShooterPIDF(1700);
                if (actionTimer.getElapsedTimeSeconds() >= initialCloseChargeTime) {
                  //  shooter.hood.setPosition(0.4);
                  //  shooter.blocker.setPosition(1);
                 //   intake.frontspin.setPower(1);
                 //   intake.backspin1.setPower(1);
                 //   intake.backspin2.setPower(-1);
                    autoTurretTargetTicks = 320;
                    setPathState(112);
                }
                break;

            case 112:
                //shooter.updateShooterPIDF(1700);
              //  shooter.hood.setPosition(0.4);
              //  shooter.blocker.setPosition(1);
               // intake.frontspin.setPower(1);
               // intake.backspin1.setPower(1);
               // intake.backspin2.setPower(-1);

                setPathState(67);

                break;

            case 67:
                if (!follower.isBusy()) {
                  //  intake.frontspin.setPower(1);
                   // intake.backspin1.setPower(1);
                  //  intake.backspin2.setPower(-1);
                   // shooter.blocker.setPosition(0);
                    autoTurretTargetTicks=0;
                    follower.followPath(grabPickup2);
                    setPathState(123);
                }
                break;

            case 123:
                if (!follower.isBusy()) {
                    //  intake.frontspin.setPower(1);
                    // intake.backspin1.setPower(1);
                    //  intake.backspin2.setPower(-1);
                    // shooter.blocker.setPosition(0);
                    autoTurretTargetTicks = 0;
                    follower.followPath(scorePickup2);
                    setPathState(-1);
                } break;

            case 87:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();

                    //shooter.updateShooterPIDF(0);
                    setPathState(345);
                }
                break;
            case 345:
                //shooter.updateShooterPIDF(0);
                if (actionTimer.getElapsedTimeSeconds() >= intakeCollectDuration) {
                  //  intake.frontspin.setPower(0);
                  //  intake.backspin1.setPower(0);
                  //  intake.backspin2.setPower(0);
                  //  shooter.blocker.setPosition(1);
                    follower.followPath(scoreregurgitate2);
                    setPathState(57);
                }
                break;

            case 57:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    //shooter.updateShooterPIDF(1700);
                    autoTurretTargetTicks=320;
                    setPathState(94);
                }
                break;

            case 94:
                //shooter.updateShooterPIDF(1700);
                if (actionTimer.getElapsedTimeSeconds() >= initialCloseChargeTime) {
                    actionTimer.resetTimer();
                    shootStep = 0;
                    setPathState(3487);
                }
                break;
            case 3487:
                //shooter.updateShooterPIDF(1700);
             //   shooter.hood.setPosition(0.4);
              //  shooter.blocker.setPosition(1);
               // intake.frontspin.setPower(1);
               // intake.backspin1.setPower(1);
              //  intake.backspin2.setPower(-1);

                setPathState(900);

                break;
            case 900:
                if (!follower.isBusy()) {
                 //   intake.frontspin.setPower(0);
                  //  intake.backspin1.setPower(0);
                  //  intake.backspin2.setPower(0);
                    autoTurretTargetTicks=0;
                    follower.followPath(parking);
                    setPathState(-1);
                }
                break;


            case -1:
                //shooter.updateShooterPIDF(0);
              //  intake.frontspin.setPower(0);
              //  intake.backspin1.setPower(0);
              //  intake.backspin2.setPower(0);
                break;

            default:
                //shooter.updateShooterPIDF(0);
              //  intake.frontspin.setPower(0);
              //  intake.backspin1.setPower(0);
              //
                //  intake.backspin2.setPower(0);
                break;
        }
    }
    // =====================================
// AUTO TURRET PID UPDATE
// =====================================

  //  public void updateAutoTurretPID() {

     //   double currentPosition = shooter.turretspin.getCurrentPosition();
     //   double error = autoTurretTargetTicks - currentPosition;

    //  double dt = autoTurretTimer.getElapsedTimeSeconds();
      //  if (dt <= 0.0) dt = 1e-3;

        //autoTurretIntegral += error * dt;
        //double derivative = (error - autoTurretLastError) / dt;

        //autoTurretLastError = error;
        //autoTurretTimer.resetTimer();

        //double output =
          //      (auto_kP * error) +
            //            (auto_kI * autoTurretIntegral) +
              //          (auto_kD * derivative);

      //  double feedforward = 0.0;
        //if (Math.abs(error) > 3) {
          //  feedforward = auto_kS * Math.signum(error);
        //}

//        output += feedforward;

  //      if (output > autoTurretMaxPower) output = autoTurretMaxPower;
    //    if (output < -autoTurretMaxPower) output = -autoTurretMaxPower;

      //  shooter.turretspin.setPower(output);
    }

