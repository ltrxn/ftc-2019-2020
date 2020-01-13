package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.System.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "auto")
@Disabled
public class Autonomous extends LinearOpMode {

    Hardware robot = new Hardware();
//    Drive drive = new Drive(robot, 100);

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Initialization", "In Progress");

        telemetry.setMsTransmissionInterval(100);
        telemetry.update();

        robot.init(hardwareMap, telemetry, this);

        telemetry.addData("Initialization", "Ready");
        telemetry.update();

        waitForStart();


    }

    public void driveStraightDistance(double distanceInInches, double masterPower) {
        int COUNTS_PER_INCH = 100;

        //target encoder count
        int targetTick = (int) (distanceInInches * COUNTS_PER_INCH);

        //used to keep track of current encoder count
        int totalTicks = 0;

        //start weaker wheel with less power
        double slavePower = masterPower - .1;

        int error = 0;
        int kp = 3;

        robot.resetEncoders();
        sleep(100);

        while ((totalTicks < targetTick) && opModeIsActive()) {

            //set motor power
            robot.setMotorPower(masterPower, slavePower, masterPower, slavePower);

            //calculate error
            //the difference between left front wheel and right front wheel
            error = robot.leftFront.getCurrentPosition() - robot.rightFront.getCurrentPosition();

            //calculate new power for slave wheel (right side)
            slavePower += error / kp;

            telemetry.addData("Error", error);
            telemetry.addData("kp", kp);
            telemetry.addData("slavePower", slavePower);

            //reset the encoders
            robot.resetEncoders();
            //drive for 100 ms
            sleep(100);

            //add to totalTicks to keep track of position
            totalTicks += robot.leftFront.getCurrentPosition();
            telemetry.addLine()
                .addData("Current Position", totalTicks)
                .addData("Target Position", targetTick);
            telemetry.update();
        }

        //stop motors once target is reached
        robot.setMotorPower(0, 0, 0, 0);

    }



}
