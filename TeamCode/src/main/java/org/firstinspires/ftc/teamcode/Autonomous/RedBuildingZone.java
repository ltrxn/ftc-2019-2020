package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.System.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Building Zone", group = "auto")
public class RedBuildingZone extends LinearOpMode {

    Hardware robot = new Hardware();

    final double DRIVE_SPEED = 0.4;
    final double DRIVE_SPEED_SLOWER = 0.30;


    @Override
    public void runOpMode() throws InterruptedException {

        /**********************
            INITIALIZATION
         **********************/
        telemetry.addData("Initialization", "In Progress");

        telemetry.setMsTransmissionInterval(100);
        telemetry.update();

        robot.init(hardwareMap, telemetry, this);

        telemetry.addData("Initialization", "Ready");
        telemetry.update();

        waitForStart();

        /**********************
         BEGIN
         **********************/
        telemetry.addData("Drive Under Bridge", "In Progress");
        telemetry.update();
        robot.setMotorPower(DRIVE_SPEED,DRIVE_SPEED_SLOWER,DRIVE_SPEED,DRIVE_SPEED_SLOWER);
        sleep(2000);

        telemetry.addData("Drive Under Bridge", "Complete");
        telemetry.update();
        robot.setMotorPower(0,0,0,0);
    }




}
