package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



import com.qualcomm.hardware.bosch.BNO055IMU;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


// import com.acmerobotics.dashboard.FtcDashboard;


@TeleOp(name="Driver control catz are cool", group="TeleOp")
public class DriverControl extends OpMode {
    private ElapsedTime runtime = new ElapsedTime(); //clock

    Hardwarerobot robot = new Hardwarerobot();
    double slowfactor = 0.5;
    static final double ARM_POWER_LIMIT = .5;
    static double CLAW_OPENED_POSITION = .08; // flip closed and open
    static double CLAW_CLOSED_POSITION = 0;
    static double ARM_COUNTS_PER_INCH = 80; //Figure out right number //114.75
    static double ARMS_COUNT_PER_ANGLE = 7.7394;
    int newTarget = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready");

        robot.armExtendorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armExtendorL.setTargetPosition(newTarget);
        robot.armExtendorL.setPower(0);
        robot.armExtendorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armExtendorR.setTargetPosition(newTarget);
        robot.armExtendorR.setPower(0);
        robot.armFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armFlipper.setTargetPosition(newTarget);
        robot.armFlipper.setPower(1);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double armExtendorPower;
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double extend = -gamepad1.right_stick_y;
        double strafe = gamepad1.left_stick_x;
        double addToClaw = .0001;
        double speed;
        double angle;


        if (gamepad2.a) {
            robot.clawL.setPosition(CLAW_CLOSED_POSITION);
            robot.clawR.setPosition(CLAW_CLOSED_POSITION);
        }
        if (gamepad2.b) {
            robot.clawL.setPosition(CLAW_OPENED_POSITION);
            robot.clawR.setPosition(CLAW_OPENED_POSITION);
        }

        if (-gamepad2.left_stick_y > .1) { //me when go up
            newTarget = (int) (newTarget + 5 * -gamepad2.left_stick_y);
            if (newTarget / ARM_COUNTS_PER_INCH > 35.5) {
                newTarget = (int) (35.5 * ARM_COUNTS_PER_INCH);
            }
            robot.armExtendorL.setTargetPosition(newTarget);
            robot.armExtendorR.setTargetPosition(newTarget);
        }
        if (gamepad2.left_stick_y > .1) {  //go down hehe
            newTarget = (int) (newTarget - 5 * gamepad2.left_stick_y);

            robot.armExtendorL.setTargetPosition(newTarget);
            robot.armExtendorR.setTargetPosition(newTarget);
        }
        if (gamepad2.x) {
            robot.armExtendorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armExtendorL.setTargetPosition(0);
            robot.armExtendorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armExtendorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armExtendorR.setTargetPosition(0);
            robot.armExtendorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.y){
            goToDrop();
        }

        if (gamepad2.dpad_down) {
            goTo0();
        } else if (gamepad2.dpad_left) {
            goTo1();
        } else if (gamepad2.dpad_right) {
            goTo2();
        } else if (gamepad2.dpad_up) {
            goTo3();
        }

        if (gamepad2.left_bumper){
            goToFlipper(130);
            closeClaw();
        }
        if (gamepad2.right_bumper){
            closeClaw();
            goToFlipper(1);
        }

        //haha look see? this is so beautiful. you smell. <3


        //armExtendorPower = Range.scale(extend,-1 ,1,-ARM_POWER_LIMIT,ARM_POWER_LIMIT);
        leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        rightBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        if (gamepad1.right_trigger > 0) {
            leftFrontPower = leftFrontPower * slowfactor;
            leftBackPower = leftBackPower * slowfactor;
            rightFrontPower = rightFrontPower * slowfactor;
            rightBackPower = rightBackPower * slowfactor;
        }

        //gotta go fast haha wait just kidding

        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.rightBackDrive.setPower(rightBackPower);
        //robot.armExtendor.setPower(armExtendorPower);
        robot.armExtendorL.setPower(Math.abs(1));
        robot.armExtendorR.setPower(Math.abs(1));
        telemetry.addData("Status", "HELLO!");
        telemetry.addData("Status", "DON'T GET ANY PENALTIES OR ELSE...");
        telemetry.addData("Status","I'M GOING TO LEAVE THE END OF THIS SENTENCE UP FOR INTERPRETATION");
        telemetry.addData("Status", "FEEL FREE TO BE CREATIVE");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("claw", "open (%.2f), closed (%.2f)", CLAW_OPENED_POSITION, CLAW_CLOSED_POSITION);
        telemetry.addData("arm", "arm_status (%.2f)", robot.armExtendorL.getCurrentPosition() / ARM_COUNTS_PER_INCH);
        telemetry.addData("arm", "arm_status (%.2f)", robot.armExtendorR.getCurrentPosition() / ARM_COUNTS_PER_INCH);
        telemetry.addData("servo position", robot.clawL.getPosition());
        telemetry.addData("servo position", robot.clawR.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
    //Slay!!!!!!!!!!

    public void goTo0() {
        double distance = 0;
        robot.armExtendorL.setPower(.5);
        robot.armExtendorR.setPower(.5);
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendorL.setTargetPosition(newTarget);
        robot.armExtendorR.setTargetPosition(newTarget);
        closeClaw();
        goToFlipper(1);
    }
    public void goToHeight(double distance) {

        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendorR.setTargetPosition(newTarget);
        robot.armExtendorL.setTargetPosition(newTarget);
    }

    public void goTo1() {
        double distance = 5;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendorL.setTargetPosition(newTarget);
        robot.armExtendorR.setTargetPosition(newTarget);
        goToDrop();
    }

    public void goTo2() {
        double distance = 15;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendorL.setTargetPosition(newTarget);
        robot.armExtendorR.setTargetPosition(newTarget);
        goToFlipper(160);
    }

    public void goTo3() {
        double distance = 30;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.armExtendorL.setTargetPosition(newTarget);
        robot.armExtendorR.setTargetPosition(newTarget);
        goToFlipper(175);
    }

    public void goToDrop() {
        robot.armFlipper.setPower(.5);
        double angles = 180;
        int armTarget = (int) (angles * ARMS_COUNT_PER_ANGLE);
        robot.armFlipper.setTargetPosition(armTarget);
    }

    public void goToFlipper(double distance) {
        robot.armFlipper.setPower(.4);
        newTarget = (int) (distance * ARMS_COUNT_PER_ANGLE);
        robot.armFlipper.setTargetPosition(newTarget);
    }
    public void closeClaw() {
        robot.clawR.setPosition(CLAW_CLOSED_POSITION);
        robot.clawL.setPosition(CLAW_CLOSED_POSITION);
    }
    public void openClaw() {
        robot.clawR.setPosition(CLAW_OPENED_POSITION);
        robot.clawL.setPosition(CLAW_OPENED_POSITION);
    }

}