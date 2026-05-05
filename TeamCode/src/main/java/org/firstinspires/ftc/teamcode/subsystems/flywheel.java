package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class flywheel {
    private DcMotorEx shooter, shooter2;


    public flywheel(DcMotorEx shooter, DcMotorEx shooter2) {
        this.shooter = shooter;
        this.shooter2 = shooter2;
    }
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void bothMotorsSetPower(double velocity) {
        shooter.setVelocity(velocity);
        shooter2.setVelocity(velocity);
    }

    public void stop() {
        shooter.setVelocity(0);
        shooter2.setVelocity(0);
    }

    public void bangBang(double targetVelocity) {
        if (shooter.getVelocity() < targetVelocity) {
            bothMotorsSetPower(6000);
        } else {
            bothMotorsSetPower(0);
        }
    }
}
