package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class turret extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this

    public int angletotick;

    private DcMotorEx turret;
    public void Turret() {
        turret = hardwareMap.get(DcMotorEx.class, "outA");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
//        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public class TurretToPosition{
        private int targetPosition;
        private double power;



    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        Turret();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();




        telemetry.addData("position", follower.getPose());
        telemetry.addData("turret", turret.getCurrentPosition());
        telemetry.addData("turretvel", turret.getVelocity());
        telemetry.update();
    }
}