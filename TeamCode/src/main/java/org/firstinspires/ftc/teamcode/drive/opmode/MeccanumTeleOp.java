/*
Copyright 2023 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
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

    //constant values
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

            denominator = Math.max(Math.abs(y) + Math.abs(x) * Math.abs(rx),1);

            fl.setPower((y + x + rx)/denominator);
            bl.setPower((y - x + rx)/denominator);
            fr.setPower((y - x - rx)/denominator);
            br.setPower((y + x - rx)/denominator);

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

        }
    }
}
