package org.firstinspires.ftc.teamcode.System;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {

    /*  Public Members  */
    public DcMotor leftFront    = null;
    public DcMotor leftBack     = null;
    public DcMotor rightFront   = null;
    public DcMotor rightBack    = null;

    public DcMotor pulley       = null;

    public Servo claw           = null;
    public BNO055IMU imu = null;

    /*  Private Members */
    private HardwareMap map           = null;
    private Telemetry tele            = null;
    private ElapsedTime period  = new ElapsedTime();
    private LinearOpMode opMode;


    /**
     * Constructor
     */
    public Hardware(){
    }

    /**
     * Initialize standard hardware interfaces
     * @param hm - Hardware map
     */
    public void init (HardwareMap hm, Telemetry t, LinearOpMode opMode) {
        //assign hardware map
        map = hm;
        tele = t;

        //define and initialize motors
        leftFront = map.get(DcMotor.class, "leftFront");
        rightFront = map.get(DcMotor.class, "rightFront");
        leftBack = map.get(DcMotor.class, "leftBack");
        rightBack = map.get(DcMotor.class, "rightBack");

        pulley = map.get(DcMotor.class, "pulley");

        //define and initialize servos
        claw = map.get(Servo.class, "claw");

        //initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.useExternalCrystal = true;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag = "IMU";
        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        this.opMode = opMode;

        //reverse left side
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

    }

    /**
     * Set the motor mode for all motors
     * @param runMode - run mode to change to.
     */
    public void setMotorMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightFront.setMode(runMode);
        rightBack.setMode(runMode);
    }

    public void resetEncoders() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the power to each motor
     * @param leftFrontPower    power to use for left front
     * @param rightFrontPower   power to use for right front
     * @param leftBackPower     power to use for left back
     * @param rightBackPower    power to use for right back
     */
    public void setMotorPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    public void leftDrive(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
    }

    public void rightDrive(double power) {
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    /**
     * Adds telemetry data about the current encoder values of the motors
     */
    public void displayEncoderValues() {
        tele.addData("Front",    "left (%.2f), right (%.2f)", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
        tele.addData("Back",     "left (%.2f), right (%.2f)", leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
    }

    public Orientation getAngles() {
        Orientation angles;
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles;
    }

    public int getAngleDegree() {
        Orientation angles = null;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int) -angles.firstAngle;
    }

    public Telemetry getTele() {
        return tele;
    }

    public void turn(int target, double speed) {
        int currentAngle = getAngleDegree(); //robot's current angel
        int turnTarget = target + currentAngle; //set the target

        if (turnTarget > 179) {
            turnTarget %= 180; //make sure you don't go >180
            turnTarget = -180 + turnTarget;
            while ((Math.abs(getAngleDegree() - turnTarget) > 3) && opMode.opModeIsActive()) {
                leftDrive(speed);
                rightDrive(0);
                tele.addData("current position", getAngleDegree());
                tele.addData("target position", turnTarget);
                tele.addData("difference between current and target: ", Math.abs(getAngleDegree() - turnTarget));
                tele.addData("is current greater? ", getAngleDegree() > target);
                tele.update();
            }

        } else if (turnTarget < -179) {
            turnTarget %= 180; //make sure you don't go <-180
            turnTarget = 180 + turnTarget;
            while ((Math.abs(getAngleDegree() - turnTarget) > 3) && opMode.opModeIsActive()) {

                leftDrive(0);
                rightDrive(speed);
                tele.addData("current position", getAngleDegree());
                tele.addData("target position", turnTarget);
                tele.addData("difference between current and target: ", Math.abs(getAngleDegree() - turnTarget));
                tele.addData("is current greater? ", getAngleDegree() > target);
                tele.update();
            }
        } else {

            while ((Math.abs(getAngleDegree() - turnTarget) > 3) && opMode.opModeIsActive()) {
                if (getAngleDegree() > target) { //if you are to the right of the target, rotate left
                    leftDrive(0);
                    rightDrive(speed);
                }
                if (getAngleDegree() < target) { //if you are to the left of the target, rotate right
                    leftDrive(speed);
                    rightDrive(0);
                }
                tele.addData("current position", getAngleDegree());
                tele.addData("target position", turnTarget);
                tele.addData("difference between current and target: ", Math.abs(getAngleDegree() - turnTarget));
                tele.addData("is current greater? ", getAngleDegree() > target);
                tele.addData("is -70>-130: ", -70 > -130);
                tele.update();
            }
        }
        leftDrive(0); //stop driving once target is reached
        rightDrive(0);
    }

}

