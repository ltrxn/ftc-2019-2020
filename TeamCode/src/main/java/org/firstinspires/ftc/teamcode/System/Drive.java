package org.firstinspires.ftc.teamcode.System;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {

    //Other hardware members
    public BNO055IMU imu;

    //hardware
//    Hardware robot;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;


    //counts per inch
    double COUNTS_PER_INCH;

    //drive variables
    double targetInches = 0;
    double masterPower = 0;
    double slavePower = 0;
    double totalTicks = 0;
    double targetTick = 0;

    int error = 0;
    final int kp = 5;

    /**
     * Constructor for Drive
     * @param hw                hardware map
     * @param COUNTS_PER_INCH   number of encoder ticks to travel one inch
     */
    public Drive (Hardware hw, double COUNTS_PER_INCH) {
//        robot = hw;

        this.imu = imu;

        this.COUNTS_PER_INCH = COUNTS_PER_INCH;

    }

    /**
     * Sets the target distance and speed
     * @param distanceInInches  distance to travel in inches
     * @param masterPower       speed
     */
    public void setDistance(double distanceInInches, double masterPower) {
        targetInches = distanceInInches;    //set distance
        this.masterPower = masterPower;    //set speed
        this.slavePower = this.masterPower-0.1;
        targetTick = (int)(distanceInInches * COUNTS_PER_INCH);

        totalTicks = 0; //reset ticks
        error = 0;      //reset error

//        robot.resetEncoders();  //reset encoders
    }

    /**
     * returns the power of slave wheels
     * @return  power to set slave wheels
     */
    public double getSlavePower() {
        slavePower += error/kp;
        return slavePower;
    }

    public double getMasterPower() { return masterPower; };

    /**
     * Calculates error while checking to see if target distance is reached
     * @return  whether target is reached
     */
    public boolean targetNotReached(int leftPosition, int rightPosition) {
//        error = robot.leftFront.getCurrentPosition() - robot.rightFront.getCurrentPosition();
        error = leftPosition - rightPosition;
        totalTicks += leftPosition;
//        robot.resetEncoders();
        return totalTicks<targetTick;

    }
/*
    public void driveStraightDistance(double distanceInInches, double masterPower) throws InterruptedException {
        int targetTick = (int)(distanceInInches * COUNTS_PER_INCH);


        int totalTicks = 0;

        //start weaker wheel with less power
        double slavePower = masterPower-.1;

        int error = 0;
        int kp = 5;

        robot.resetEncoders();
        sleep(100);

        while(totalTicks<targetTick) {
            robot.setMotorPower(masterPower, slavePower, masterPower, slavePower);
            error = leftFront.getCurrentPosition() - rightFront.getCurrentPosition();

            slavePower += error/kp;

            robot.resetEncoders();
            sleep(100);

            totalTicks+=leftFront.getCurrentPosition();
        }
        robot.setMotorPower(0,0,0,0);
    }

 */

    public void turn(int degrees) {

    }


}
