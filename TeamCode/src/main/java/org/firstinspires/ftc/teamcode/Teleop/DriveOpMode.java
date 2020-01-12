package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.System.Hardware;

import java.util.Locale;

@TeleOp(name="Basic Drive", group="Teleop")
public class DriveOpMode extends LinearOpMode {

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    //servos
    private static final double CLAW_OPEN   = .65;
    private static final double CLAW_CLOSE  = .15;
    private double clawPosition = CLAW_OPEN;

    //motor
    private double pulleyPower = 0;


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

/*
            //FIELD CENTRIC DRIVING
            //Gamepad 1 - LEFT JOYSTICK - Strafes robot
            double findRadius = Math.hypot(-gamepad1.right_stick_x, -gamepad1.right_stick_y);
            double findRadian = (Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 3.5);
            double findDegree = (findRadian*(180/Math.PI)) + robot.getAngleDegree();
            double findAngle = findDegree*(Math.PI/180);
            double leftY = gamepad1.left_stick_y / 1.2;
            double v1 = findRadius * Math.cos(findAngle) - leftY;
            double v2 = findRadius * Math.sin(findAngle) - leftY;
            double v3 = findRadius * Math.sin(findAngle) - leftY;
            double v4 = findRadius * Math.cos(findAngle) - leftY;

            telemetry.addData("Radius", findRadius);
            telemetry.addData("Angle", findAngle);

 */


            //Gamepad 1 - LEFT JOYSTICK - Strafes robot
            double radius = Math.hypot(-gamepad1.right_stick_x, -gamepad1.right_stick_y);
            double robotAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/ 3.5;
            double leftY = -gamepad1.left_stick_y / 2;
            double v1 = radius * Math.cos(robotAngle) + leftY;
            double v2 = radius * Math.sin(robotAngle) + leftY;
            double v3 = radius * Math.sin(robotAngle) + leftY;
            double v4 = radius * Math.cos(robotAngle) + leftY;



            //Gamepad 1 - RIGHT TRIGGER - Robot turns clockwise
            if (gamepad1.right_trigger > 0) {
                double speed = scaleInput(gamepad1.right_trigger);
                v1 = speed;
                v2 = -speed;
                v3 = speed;
                v4 = -speed;
            }

            //Gamepad 1 - LET TRIGGER - Robot turns counter-clockwise
            if (gamepad1.left_trigger > 0) {
                double speed = scaleInput(gamepad1.left_trigger);
                v1 = -speed;
                v2 = speed;
                v3 = -speed;
                v4 = speed;
            }

            //Gamepad 1/2 - RIGHT BUMPER - Claws open
            if (gamepad1.left_bumper || gamepad2.left_trigger > 0.2) {
                clawPosition = CLAW_OPEN;
            }

            //Gamepad 1/2 - LEFT BUMPER - Claws closes
            else if (gamepad1.right_bumper || gamepad2.right_trigger > 0.2) {
                clawPosition = CLAW_CLOSE;
            }


            //set power to servos and motors
            robot.claw.setPosition(clawPosition);
            robot.setMotorPower(v1, v2, v3, v4);
            robot.pulley.setPower(-gamepad2.right_stick_y);

            //telemetry

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Encoder rightFront", robot.rightFront.getCurrentPosition());
            telemetry.addData("Encoder rightBack", robot.rightBack.getCurrentPosition());
            telemetry.addData("Encoder leftFront", robot.leftFront.getCurrentPosition());
            telemetry.addData("Encoder leftBack", robot.leftBack.getCurrentPosition());
            telemetry.addData("Heading", formatAngle(robot.getAngles().angleUnit, robot.getAngles().firstAngle));
            telemetry.addData("Claw position", robot.claw.getPosition());
            telemetry.update();
            idle();

        }

    }

    //Makes joystick easier to control
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}
