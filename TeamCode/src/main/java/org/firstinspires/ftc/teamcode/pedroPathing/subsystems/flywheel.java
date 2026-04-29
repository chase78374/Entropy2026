package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class flywheel {
    private DcMotorEx shooter, shooter2;

    public flywheel(DcMotorEx shooter, DcMotorEx shooter2) {
        this.shooter = shooter;
        this.shooter2 = shooter2;
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
