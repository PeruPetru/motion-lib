package org.firstinspires.ftc.teamcode.pathfollower;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pathfollower.datastructures.Pose;
import org.firstinspires.ftc.teamcode.pathfollower.localization.Localizer;
import org.firstinspires.ftc.teamcode.pathfollower.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pathfollower.paths.BezierPath;
import org.firstinspires.ftc.teamcode.pathfollower.paths.Path;
import org.firstinspires.ftc.teamcode.util.Motor;
import org.firstinspires.ftc.teamcode.util.PDSGCoefficients;
import org.firstinspires.ftc.teamcode.util.PDSGController;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;


@Config
public class MecanumDrive {
    public static double MAX_VELOCITY = 100;
    public static boolean FOLLOW_HEADING_TANGENT_TO_PATH = false;
    public static boolean REVERSE_TANGENT_FOLLOWING = false;

    private final double LENGTH = 44;
    private final double WIDTH = 32;

    public static PDSGCoefficients FORWARD_COEFFICIENTS = new PDSGCoefficients(0, 0, 0, 0, 0);
    public static PDSGCoefficients LATERAL_COEFFICIENTS = new PDSGCoefficients(0, 0, 0, 0, 0);
    public static PDSGCoefficients HEADING_COEFFICIENTS = new PDSGCoefficients(0, 0, 0, 0, 0);

    private Localizer localizer;

    public enum DriveMode{
        AUTO,
        MANUAL
    }

    List<Path> paths = new ArrayList<Path>();
    double lastUpdateTime = 0;
    public static Pose targetPose = new Pose();
    public static double headingTarget;
    public static PDSGController forwardController = new PDSGController(FORWARD_COEFFICIENTS);
    public static PDSGController lateralController = new PDSGController(LATERAL_COEFFICIENTS);
    public static PDSGController headingController = new PDSGController(HEADING_COEFFICIENTS);
    public static double K_STATIC = 0.0;

    public Motor leftFront;
    public Motor rightFront;
    public Motor leftRear;
    public Motor rightRear;

    private Supplier<Double> voltageSupplier;

    public static DriveMode driveMode = DriveMode.MANUAL;

    public MecanumDrive(HardwareMap hardwareMap, Supplier<Double> voltageSupplier){
        this.leftFront = new Motor(hardwareMap.get(DcMotorEx.class, "leftFront"));
        this.rightFront = new Motor(hardwareMap.get(DcMotorEx.class, "rightFront"));
        this.leftRear = new Motor(hardwareMap.get(DcMotorEx.class, "leftBack"));
        this.rightRear = new Motor(hardwareMap.get(DcMotorEx.class, "rightBack"));

        rightFront.setReversed(false);
        rightRear.setReversed(false);

        this.voltageSupplier = voltageSupplier;

        localizer = new PinpointLocalizer(hardwareMap);
    }

    public void splineTo(Pose p2, Pose p3, Pose endPose){
        paths.add(new BezierPath(paths.isEmpty()?getPose():paths.get(paths.size()-1).get(1), p2, p3, endPose));
    }

    public void splineTo(Pose p2, Pose endPose){
        if(p2 == null){
            splineTo(endPose);
            return;
        }
        paths.add(new BezierPath(paths.isEmpty()?getPose():paths.get(paths.size()-1).get(1), p2, p2, endPose));
    }

    public void splineTo(Pose endPose){
        paths.add(new BezierPath(paths.isEmpty()?getPose():paths.get(paths.size()-1).get(1), endPose, endPose, endPose));
    }
    public void clearSplines(){
        paths.clear();
        targetPose = getPose();
    }

    public Pose getPose(){
        return localizer.getPose();
    }

    public boolean atTarget(){
        return  Math.abs(targetPose.x - getPose().x) < 3 &&
                Math.abs(targetPose.y - getPose().y) < 3 &&
                AngleUnit.normalizeRadians(Math.abs(targetPose.heading - getPose().heading)) < 0.07;
    }

    public boolean atTarget(int tolerance) {
        return Math.abs(targetPose.x - getPose().x) < tolerance &&
                Math.abs(targetPose.y - getPose().y) < tolerance &&
                AngleUnit.normalizeRadians(Math.abs(targetPose.heading - getPose().heading)) < 0.07;
    }

    public boolean isFinished() {
        return isFinished(3);
    }

    public boolean isFinished(int tolerance){
        return paths.isEmpty() && atTarget(tolerance);
    }

    public void setMaxVelocity(double velocity){
        MAX_VELOCITY = velocity;
    }

    public void driveFieldCentric(double forward, double strafe, double rotate){
        double heading = localizer.getPose().heading;
        driveRobotCentric(forward*Math.cos(heading)-strafe*Math.sin(heading),
                          forward*Math.sin(heading)+strafe*Math.cos(heading),
                          rotate);
    }

    public void driveRobotCentric(double forward, double strafe, double rotate){
        double denominator = Math.max(1, forward+strafe+rotate);
        forward = forward / denominator;
        strafe = strafe / denominator;
        rotate = rotate / denominator;
        double leftFrontPower = forward + strafe - rotate;
        double rightFrontPower = forward - strafe + rotate;
        double leftRearPower = forward - strafe - rotate;
        double rightRearPower = forward + strafe + rotate;
        double staticFeedforward = (driveMode == DriveMode.AUTO && !atTarget(1) ? K_STATIC :  0);
        leftFront.setPower( leftFrontPower +  staticFeedforward * Math.signum(leftFrontPower) );
        rightFront.setPower( rightFrontPower + staticFeedforward * Math.signum(rightFrontPower) );
        leftRear.setPower( leftRearPower + staticFeedforward * Math.signum(leftRearPower) );
        rightRear.setPower( rightRearPower + staticFeedforward * Math.signum(rightRearPower) );
    }


    public void setPose(Pose pose){
        localizer.setPose(pose);
    }

    public void followTangentially(boolean value, boolean reversed){
        FOLLOW_HEADING_TANGENT_TO_PATH = value;
        REVERSE_TANGENT_FOLLOWING = reversed;
    }

    public void followTangentially(boolean value){
        FOLLOW_HEADING_TANGENT_TO_PATH = value;
    }

    public void draw(TelemetryPacket packet){
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#000000af");
        fieldOverlay.strokePolygon(new double[]{
                (getPose().x + LENGTH / 2 * Math.cos(getPose().heading) - WIDTH / 2 * Math.sin(getPose().heading)) / 2.54,
                (getPose().x - LENGTH / 2 * Math.cos(getPose().heading) - WIDTH / 2 * Math.sin(getPose().heading)) / 2.54,
                (getPose().x - LENGTH / 2 * Math.cos(getPose().heading) + WIDTH / 2 * Math.sin(getPose().heading)) / 2.54,
                (getPose().x + LENGTH / 2 * Math.cos(getPose().heading) + WIDTH / 2 * Math.sin(getPose().heading)) / 2.54
            },
            new double[]{
                (getPose().y + LENGTH / 2 * Math.sin(getPose().heading) + WIDTH / 2 * Math.cos(getPose().heading)) / 2.54,
                (getPose().y - LENGTH / 2 * Math.sin(getPose().heading) + WIDTH / 2 * Math.cos(getPose().heading)) / 2.54,
                (getPose().y - LENGTH / 2 * Math.sin(getPose().heading) - WIDTH / 2 * Math.cos(getPose().heading)) / 2.54,
                (getPose().y + LENGTH / 2 * Math.sin(getPose().heading) - WIDTH / 2 * Math.cos(getPose().heading)) / 2.54
            }
        );
        fieldOverlay.strokeCircle(getPose().x / 2.54 ,getPose().y / 2.54, 0.5 / 2.54);
        fieldOverlay.strokeLine(getPose().x / 2.54, getPose().y / 2.54,
                getPose().x / 2.54 + Math.cos(getPose().heading) * 16 / 2.54, getPose().y / 2.54 + Math.sin(getPose().heading) * 16 / 2.54);

        if(driveMode != DriveMode.AUTO) return;

        fieldOverlay.setStroke("#f03030af");
        fieldOverlay.strokePolygon(new double[]{
                (targetPose.x + LENGTH / 2 * Math.cos(targetPose.heading) - WIDTH / 2 * Math.sin(targetPose.heading)) / 2.54,
                (targetPose.x - LENGTH / 2 * Math.cos(targetPose.heading) - WIDTH / 2 * Math.sin(targetPose.heading)) / 2.54,
                (targetPose.x - LENGTH / 2 * Math.cos(targetPose.heading) + WIDTH / 2 * Math.sin(targetPose.heading)) / 2.54,
                (targetPose.x + LENGTH / 2 * Math.cos(targetPose.heading) + WIDTH / 2 * Math.sin(targetPose.heading)) / 2.54
            },
            new double[]{
                (targetPose.y + LENGTH / 2 * Math.sin(targetPose.heading) + WIDTH / 2 * Math.cos(targetPose.heading)) / 2.54,
                (targetPose.y - LENGTH / 2 * Math.sin(targetPose.heading) + WIDTH / 2 * Math.cos(targetPose.heading)) / 2.54,
                (targetPose.y - LENGTH / 2 * Math.sin(targetPose.heading) - WIDTH / 2 * Math.cos(targetPose.heading)) / 2.54,
                (targetPose.y + LENGTH / 2 * Math.sin(targetPose.heading) - WIDTH / 2 * Math.cos(targetPose.heading)) / 2.54
            }
        );
        for (Path path : paths) {
            ArrayList<Double> xPoints = new ArrayList<>();
            ArrayList<Double> yPoints = new ArrayList<>();
            Pose lastPose;
            double t = 2;
            lastPose = path.get(0);
            while(path.get(t) != lastPose){
                xPoints.add(lastPose.x / 2.54);
                yPoints.add(lastPose.y / 2.54);
                lastPose = path.get(t);
                t += 2;
            }
            fieldOverlay.setStroke("#30a030ff");
            fieldOverlay.strokePolyline(xPoints.stream().mapToDouble(i -> i).toArray(), yPoints.stream().mapToDouble(i -> i).toArray());
        }
    }
    public void update(TelemetryPacket packet){
        double deltaTime = System.nanoTime() / 1e9 - lastUpdateTime;
        lastUpdateTime = System.nanoTime() / 1e9;
        localizer.update(packet);
        forwardController.setPDSG_COEFFICIENTS(FORWARD_COEFFICIENTS);
        lateralController.setPDSG_COEFFICIENTS(LATERAL_COEFFICIENTS);
        headingController.setPDSG_COEFFICIENTS(HEADING_COEFFICIENTS);


        draw(packet);
        packet.put("x", getPose().x);
        packet.put("y", getPose().y);
        packet.put("heading (deg)", Math.toDegrees(getPose().heading));
        packet.put("xerr", targetPose.x - getPose().x);
        packet.put("yerr", targetPose.y - getPose().y);
        packet.put("heading (deg)err", Math.toDegrees(targetPose.heading) - Math.toDegrees(getPose().heading));


        if(driveMode != DriveMode.AUTO) return;

        if(FOLLOW_HEADING_TANGENT_TO_PATH){
            headingTarget = targetPose.heading;
        }else{
            headingTarget = targetPose.heading;
        }

        double headingError = AngleUnit.normalizeRadians( headingTarget - getPose().heading );

        forwardController.setTarget( Math.cos( -getPose().heading ) *  targetPose.x - Math.sin( -getPose().heading ) * targetPose.y );
        lateralController.setTarget( Math.sin( -getPose().heading ) * targetPose.x  + Math.cos( -getPose().heading ) * targetPose.y );

        double forwardPower = forwardController.update(getPose().x) * 12.0 / voltageSupplier.get();
        double strafingPower = - lateralController.update(getPose().y) * 12.0 / voltageSupplier.get();
        double rotatingPower = headingController.update(headingError) * 12.0 / voltageSupplier.get();

        driveRobotCentric(forwardPower, strafingPower, rotatingPower);

        if (paths.isEmpty()) return;
        if (atTarget(9) && paths.get(0).atEnd(getPose())) {
            paths.remove(0);
            if (paths.isEmpty()) return;
        }
        /*if((targetPose.x - getPose().x)*(targetPose.x - getPose().x) + (targetPose.y - getPose().y)*(targetPose.y - getPose().y) < FOLLOWER_RADIUS * FOLLOWER_RADIUS) {
            currentDistanceFromPathStart += deltaTime * MAX_VELOCITY;
            targetPose = paths.get(0).getLengthRelative(currentDistanceFromPathStart);
        }*/
    }
}

