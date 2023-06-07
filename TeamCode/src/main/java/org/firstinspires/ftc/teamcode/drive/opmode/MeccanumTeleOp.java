package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MeccanumTeleOp extends LinearOpMode {

    //declaring motors
    private DcMotor bl;
    private DcMotor br;
    private DcMotor fl;
    private DcMotor fr;

    private DcMotor lift0;
    private DcMotor lift1;

    private Servo left;
    private Servo right;

    private double y;
    private double x;
    private double rx; //right stick x
    private double denominator;

    //servo position constants
    private double rightOpen = 0.4;
    private double leftOpen = 0.7;
    private double rightClosed = .9;
    private double leftClosed = .2;

    @Override
    public void runOpMode() {
        //hardware mapping
        //claw servos
        left = hardwareMap.get(Servo.class, "claw1");
        right = hardwareMap.get(Servo.class, "claw0");

        //drive
        bl = hardwareMap.get(DcMotor.class, "Bl");
        br = hardwareMap.get(DcMotor.class, "Br");
        fl = hardwareMap.get(DcMotor.class, "Fl");
        fr = hardwareMap.get(DcMotor.class, "Fr");

        //lift
        lift0 = hardwareMap.get(DcMotor.class, "lift0");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");


        //motor directions and behavior
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //2 modes, BRAKE and FLOAT. BRAKE will use the motors internal magnetic feild to apply resistance when the power is set to 0, whereas float will let the motor freely spin.
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        left.setPosition(0.7);
        right.setPosition(0.4);


        waitForStart();

        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y; //gamepad1.(command) will check for inputs in the gamepad. to get another gamepad, use gamepad2.(command)
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            //makes sure that the power set to motor is not under -1 and over 1.
            denominator = Math.max(Math.abs(y) + Math.abs(x) * Math.abs(rx),1);

            //meccanum kinematics
            fl.setPower((y + x + rx)/denominator);
            bl.setPower((y - x + rx)/denominator);
            fr.setPower((y - x - rx)/denominator);
            br.setPower((y + x - rx)/denominator);

            //input conditions
            if(gamepad1.a) {
                lift0.setPower(.1);
                lift1.setPower(.1);
            } else if(gamepad1.b) {
                lift0.setPower(-.4);
                lift1.setPower(-.4); //setPower sets power to the motor, has to be between -1 and 1.
            }
            else if (gamepad1.left_trigger >.5) {
                left.setPosition(leftClosed); //setPosition changes the position of a servo. The value has to be between 0 and 1
                right.setPosition(rightClosed);
            } else if (gamepad1.right_trigger >.5) {
                left.setPosition(leftOpen);
                right.setPosition(rightOpen);
            } else {
                lift0.setPower(-.1);
                lift1.setPower(-.1); //some small motor power value to make sure the arm doesn't fall when unpowered
            }
            //telemetry.addData("what the data is", motorName.getCurrentPosition()); using the getCurrentPosition() function gets the current position of a motor assuming an encoder is plugged in.
            //telemtry.update(); //you can use encoder data to automate arm movements and get them to whatever position is desired.
        }
    }
}
