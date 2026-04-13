package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.flywheel;



@TeleOp(name="Entropy2026", group="Linear Opmode")
    public class Entropy2026 extends LinearOpMode {

        private DcMotor FL = null;
        private DcMotor FR = null;
        private DcMotor BL = null;
        private DcMotor BR = null;
        private DcMotorEx intake = null;
        private DcMotorEx shooter = null;
        private DcMotorEx shooter2 = null;
        private CRServo l = null;
        private CRServo r = null;
        private boolean flyWheelRunning;
    public void bothMotorsSetPower(double power) {
        shooter.setPower(power);
        shooter2.setPower(power);
    }
    private flywheel flywheelSubsystem;
    private static final double flywheelVelocity = 1200;



    @Override
        public void runOpMode() {


            intake = hardwareMap.get(DcMotorEx.class, "intake");
            shooter = hardwareMap.get(DcMotorEx.class, "shooter");
            shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
            l = (CRServo) hardwareMap.get(CRServo.class, "l");
            r = (CRServo) hardwareMap.get(CRServo.class, "r");
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
            shooter.setDirection(DcMotor.Direction.REVERSE);
            shooter2.setDirection(DcMotor.Direction.FORWARD);
            l.setDirection(CRServo.Direction.REVERSE);
            r.setDirection(CRServo.Direction.FORWARD);







        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.right_bumper) {
                    l.setPower(1);
                    r.setPower(1);
                } else if (gamepad1.left_bumper) {
                    l.setPower(-1);
                    r.setPower(-1);
                } else {
                    l.setPower(0);
                    r.setPower(0);
                }

                if (gamepad1.right_trigger>.1){
                    intake.setVelocity(1150);

                } else if (gamepad1.left_trigger>.1){
                    intake.setPower(-1);

                } else {
                    intake.setVelocity(0);
                }
                double axial   = -gamepad1.left_stick_y;
                double lateral =  gamepad1.left_stick_x;
                double yaw     =  gamepad1.right_stick_x;

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
                    flywheelSubsystem.bangBang(flywheelVelocity);
                }

                    
                    







                telemetry.addData("Motor Power", intake.getPower());
                telemetry.addData("Encoder Position", intake.getCurrentPosition());
                telemetry.addData("Motor Power", shooter.getPower());
                telemetry.addData("Encoder Position", shooter.getCurrentPosition());
                telemetry.addData("Motor Power", shooter2.getPower());
                telemetry.addData("Encoder Position", shooter2.getCurrentPosition());
                telemetry.update();

            }
        }
    }

