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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class turret extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private ElapsedTime timer = new ElapsedTime();
    public int angletotick;
    private DcMotorEx turret;
    public void Turret() {
        turret = hardwareMap.get(DcMotorEx.class, "outA");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public class TurretToPosition{
        private int targetPosition;
        private double power;



    }

    // Red goal coordinates
    private static final double RED_GOAL_X = 60;
private static final double RED_GOAL_Y = -60;

    /**
     * Gets the current robot coordinates and calculates distance to the red goal.
     * @return distance from robot to red goal at (132, -132)
     */
    public double getDistanceToRedGoal() {
        Pose currentPose = follower.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();

        double deltaX = RED_GOAL_X - robotX;
        double deltaY = RED_GOAL_Y - robotY;

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Calculates the heading offset between the robot's front and the red goal.
     * @return angle offset in radians (positive = goal is to the left, negative = goal is to the right)
     */
    public double getHeadingOffsetToRedGoal() {
        Pose currentPose = follower.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotHeading = currentPose.getHeading();

        double deltaX = RED_GOAL_X - robotX;
        double deltaY = RED_GOAL_Y - robotY;

        // Calculate absolute angle from robot to goal using atan2
        double angleToGoal = Math.atan2(deltaY, deltaX);

        // Calculate offset from robot's current heading
        double headingOffset = angleToGoal - robotHeading;

        // Normalize to [-π, π]
        while (headingOffset > Math.PI) headingOffset -= 2 * Math.PI;
        while (headingOffset < -Math.PI) headingOffset += 2 * Math.PI;

        return headingOffset;
    }

    /**
     * Calculates the heading offset in degrees.
     * @return angle offset in degrees
     */
    public double getHeadingOffsetToRedGoalDegrees() {
        return Math.toDegrees(getHeadingOffsetToRedGoal());
    }

    /**
     * Gets the current robot coordinates.
     * @return the current Pose (x, y, heading) of the robot
     */
    public Pose getRobotCoordinates() {
        return follower.getPose();
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
        timer.reset();
        //Call this once per loop
        follower.update();




        telemetry.addData("position", follower.getPose());
        telemetry.addData("turret", turret.getCurrentPosition());
        telemetry.addData("turretvel", turret.getVelocity());
        telemetry.addData("loop time", timer.time());
        telemetry.addData("dist", getDistanceToRedGoal());
        telemetry.addData("angle", getHeadingOffsetToRedGoalDegrees());
        telemetry.update();
    }
}