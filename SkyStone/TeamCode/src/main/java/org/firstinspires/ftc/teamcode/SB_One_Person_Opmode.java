package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//Importing all FTC servos, motor, hardware map and time into this opmode. These packages are made by others and are
// imported into this program. FTC has already defined it.
/**
 * Created by dhruv on 11/30/2017.
 */

public class SB_One_Person_Opmode {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor arm = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;

    public final static double ARM_HOME = 0.2;
    public final static double CLAW_HOME = 0;
    public final static double ARM_MIN_RANGE = 0.20;
    public final static double ARM_MAX_RANGE = 0.90;
    public final static double CLAW_MIN_RANGE = 0.20;
    public final static double CLAW_MAX_RANGE = 0.7;


    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();


    /* Constructor */
    public SB_One_Person_Opmode() {
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;


        // Define and Initialize Motors
        arm = hwMap.get(DcMotor.class, "arm1");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive = hwMap.get(DcMotor.class, "left_motor");
        rightDrive = hwMap.get(DcMotor.class, "right_motor");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
        rightClaw = hwMap.get(Servo.class, "right_claw");
        leftClaw = hwMap.get(Servo.class, "left_claw");
        leftClaw.setPosition(CLAW_HOME);
        rightClaw.setPosition(1);
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.FORWARD);
    }
}
