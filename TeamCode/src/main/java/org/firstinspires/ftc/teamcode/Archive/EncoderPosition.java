package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.System.Hardware;

public class EncoderPosition implements Runnable {

    //Thead run condition
    private boolean isRunning = true;

    //Motors with encoders
    public DcMotor leftEncoderWheel, rightEncoderWheel;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Position variables used for storage and calculation
    double rightEncoderWheelPosition = 0, leftEncoderWheelPosition = 0, changeInRobotOrientation = 0;
    private double robotOrientationRadians = 0;
    private double previousRightEncoderWheelPosition = 0, previousLeftEncoderWheelPosition = 0;

    //counts per inch
    double COUNTS_PER_INCH;

    public EncoderPosition(Hardware hw, double COUNTS_PER_INCH, int threadSleepDelay) {
        leftEncoderWheel = hw.leftFront;
        rightEncoderWheel = hw.rightFront;

        sleepTime = threadSleepDelay;

        this.COUNTS_PER_INCH = COUNTS_PER_INCH;
    }

    public void updatePosition() {

        //get current positions
        leftEncoderWheelPosition = leftEncoderWheel.getCurrentPosition();
        rightEncoderWheelPosition = rightEncoderWheel.getCurrentPosition();

        //

    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            updatePosition();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

}
