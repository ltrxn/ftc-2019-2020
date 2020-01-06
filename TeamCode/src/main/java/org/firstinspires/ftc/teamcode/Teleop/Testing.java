package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Archive.Drive;
import org.firstinspires.ftc.teamcode.System.Hardware;

@TeleOp(name = "Testing", group = "Teleop")
public class Testing extends LinearOpMode {
    Hardware robot = new Hardware();
    Drive drive = new Drive(robot, 100);
    double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initialization", "In Progress");

        telemetry.setMsTransmissionInterval(100);
        telemetry.update();

        robot.init(hardwareMap, telemetry, this);

        telemetry.addData("Initialization", "Ready");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {


/*TESTING CLAW
            if (gamepad1.left_bumper) {
                pos -= 0.05;
            }

            if (gamepad1.right_bumper) {
                pos += 0.05;
            }

            robot.claw.setPosition(pos);

            telemetry.addData("servo", pos);
            telemetry.update();

 */
            sleep(50);
        }
        }
    }