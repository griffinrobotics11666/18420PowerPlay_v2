package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Hardwarerobot
{
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  leftBackDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  rightBackDrive  = null;
    public DcMotor  armExtendorL  = null;
    public DcMotor  armExtendorR  = null;
    public DcMotor  armFlipper = null;
    public Servo    clawL   = null;
    public Servo    clawR   = null;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public Hardwarerobot(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        leftFrontDrive  = hwMap.get(DcMotor.class, "FL");
        leftBackDrive  = hwMap.get(DcMotor.class, "BL");
        rightFrontDrive = hwMap.get(DcMotor.class, "RF");
        rightBackDrive = hwMap.get(DcMotor.class, "RB");
        armExtendorR = hwMap.get(DcMotor.class, "RL");
        armExtendorL = hwMap.get(DcMotor.class, "LL");
        clawL = hwMap.get(Servo.class, "LC");
        clawR = hwMap.get(Servo.class, "RC");
        armFlipper = hwMap.get(DcMotor.class, "FBFL");


        clawR.setDirection(Servo.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        armExtendorL.setDirection(DcMotor.Direction.FORWARD);
        armExtendorR.setDirection(DcMotor.Direction.FORWARD);
        armFlipper.setDirection(DcMotor.Direction.FORWARD);

        armExtendorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtendorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armFlipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        armExtendorL.setPower(0);
        armExtendorR.setPower(0);
        armFlipper.setPower(0);

        leftBackDrive.setTargetPosition(0);
        leftFrontDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armExtendorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendorL.setTargetPosition(0);
        armExtendorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armFlipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armFlipper.setTargetPosition(0);
        armFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtendorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendorR.setTargetPosition(0);
        armExtendorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawL.setPosition(0);
        clawR.setPosition(0);

    }
}

