package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.System.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Loading Zone and Red Building Zone", group = "official")
public class RedBuildingZone extends LinearOpMode {

    Hardware robot = new Hardware();

    final double DRIVE_SPEED = 0.4;
    final double DRIVE_SPEED_SLOWER = 0.38;


    @Override
    public void runOpMode() throws InterruptedException {

        /**********************
            INITIALIZATION
         **********************/
        telemetry.addData("For BLUE loading zone", "or RED building zone");
        telemetry.addData("Align FRONT wheels against the tile seam!", "face AWAY from the bridge.");
        telemetry.addData("Initialization", "In Progress");

        telemetry.setMsTransmissionInterval(100);
        telemetry.update();

        robot.init(hardwareMap, telemetry, this);

        telemetry.addData("For BLUE loading zone", "or RED building zone");
        telemetry.addData("Align back wheels against the tile seam!", "face the bridge.");
        telemetry.addData("Initialization", "Ready");
        telemetry.update();

        waitForStart();

        /**********************
         BEGIN
         **********************/

        telemetry.addData("Drive Under Bridge", "In Progress");
        telemetry.update();

        //strafe to the LEFT
        robot.setMotorPower(-DRIVE_SPEED,DRIVE_SPEED_SLOWER,DRIVE_SPEED,-DRIVE_SPEED_SLOWER);
        //to travel further
        sleep(4400);
        //to travel normal
        //sleep(500);

        robot.setMotorPower(0,0,0,0);

       //move forward
        robot.setMotorPower(-DRIVE_SPEED,-DRIVE_SPEED_SLOWER,-DRIVE_SPEED,-DRIVE_SPEED_SLOWER);
        sleep(1600);

        telemetry.addData("Drive Under Bridge", "Complete");
        telemetry.update();
        robot.setMotorPower(0,0,0,0);




    }

}
