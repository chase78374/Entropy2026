package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.flywheel;



@TeleOp(name="Entropy2026", group="Linear Opmode")
public class Entropy2026 extends LinearOpMode {

    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor intake = null;
    private DcMotor intake2 = null;
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private Servo l = null;
    private Servo r = null;
    private boolean flyWheelRunning;
    private boolean rightBumper;

    public void bothMotorsSetPower(double power) {
        shooter.setPower(power);
        shooter2.setPower(power);
    }

    private flywheel flywheelSubsystem;
    private double targetVelocity = 1250;

    boolean prevDpadUp = false;
    boolean prevDpadDown = false;
    boolean prevRightBumper = false;


    @Override
    public void runOpMode() {


        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        l = (Servo) hardwareMap.get(Servo.class, "l");
        r = (Servo) hardwareMap.get(Servo.class, "r");
        flywheelSubsystem = new flywheel(shooter, shooter2);
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");




        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);










        intake.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        l.setDirection(Servo.Direction.FORWARD);
        r.setDirection(Servo.Direction.REVERSE);







        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper && !prevRightBumper) {
                rightBumper = !rightBumper;
            }
            prevRightBumper = gamepad1.right_bumper;

            l.setPosition(rightBumper ? 1.0 : 0.3);
            r.setPosition(rightBumper ? 1.0 : 0.5);

            if (gamepad1.right_trigger>.1){
                intake.setPower(1);
                intake2.setPower(1);

            } else if (gamepad1.left_trigger>.1){
                intake.setPower(-.5);
                intake2.setPower(-.5);

            } else {
                intake.setPower(0);
                intake2.setPower(0);
            }
            if (gamepad1.dpad_up && !prevDpadUp) {
                targetVelocity += 100;
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                targetVelocity -= 100;
            }
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  (gamepad1.right_stick_x)/1.2;

            double max;
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            FL.setPower(frontLeftPower);
            FR.setPower(frontRightPower);
            BL.setPower(backLeftPower);
            BR.setPower(backRightPower);


            if (gamepad1.yWasPressed()) {
                flyWheelRunning = !flyWheelRunning;
                if (!flyWheelRunning) {
                    flywheelSubsystem.stop();
                }
            }
            if (flyWheelRunning) {
                flywheelSubsystem.bangBang(targetVelocity);
            }










            telemetry.addData("Motor Power", intake.getPower());
            telemetry.addData("Encoder Position", intake.getCurrentPosition());
            telemetry.addData("Motor Power", intake2.getPower());
            telemetry.addData("Encoder Position", intake2.getCurrentPosition());
            telemetry.addData("Motor Power", shooter.getPower());
            telemetry.addData("Encoder Position", shooter.getCurrentPosition());
            telemetry.addData("Motor Power", shooter2.getPower());
            telemetry.addData("Encoder Position", shooter2.getCurrentPosition());
            telemetry.addData("flywheel speed",targetVelocity);
            telemetry.update();

        }
    }
}
