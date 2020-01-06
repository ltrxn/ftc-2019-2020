package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.System.Drive;
import org.firstinspires.ftc.teamcode.System.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "auto")
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

        int targetTick = (int) (distanceInInches * COUNTS_PER_INCH);

        int totalTicks = 0;

        //start weaker wheel with less power
        double slavePower = masterPower - .1;

        int error = 0;
        int kp = 3;

        robot.resetEncoders();
        sleep(100);

        while ((totalTicks < targetTick) && opModeIsActive()) {

            robot.setMotorPower(masterPower, slavePower, masterPower, slavePower);
            error = robot.leftFront.getCurrentPosition() - robot.rightFront.getCurrentPosition();

            slavePower += error / kp;

            telemetry.addData("Error", error);
            telemetry.addData("kp", kp);
            telemetry.addData("slavePower", slavePower);

            robot.resetEncoders();
            sleep(100);

            totalTicks += robot.leftFront.getCurrentPosition();
            telemetry.addLine()
                .addData("Current Position", totalTicks)
                .addData("Target Position", targetTick);
            telemetry.update();
        }

        robot.setMotorPower(0, 0, 0, 0);

    }



}
