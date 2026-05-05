package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem implements EntropySubsystem {

    //@Override
    public DcMotorEx intake1;
    public DcMotorEx intake2;

    public HardwareMap hardwareMap;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void teleOpManual(Gamepad gamepad) {};
    public void init() {
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setSpeed(double speed) {
        intake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1.setVelocity(speed);
        intake2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setVelocity(speed);
    }

    public void intake(String intaking) {
        if (intaking.equals( "in")) {
            setSpeed(10);
        } else if (intaking.equals ("not")){
            setSpeed(0);
        } else if (intaking.equals("out")) {
            setSpeed(-10);
        }
    }
    public void passOne() {
        intake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake1.setTargetPosition(intake1.getCurrentPosition() + 500);
        intake1.setPower(.3);
        intake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake2.setTargetPosition(intake1.getCurrentPosition() + 500);
        intake2.setPower(.3);
    }

    public void setIntakePower(double power) {
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setPower(power);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setPower(power);
    }

    public void intakeAuto() {
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setPower(1);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setPower(1);
    }

    public void intakeOffAuto() {
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setPower(0);
        intake2.setPower(0);

    }
}
