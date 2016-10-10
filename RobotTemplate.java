/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import com.sun.squawk.util.MathUtils;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    /*
     * Declare all motors, sensors, timers, etc.
     */

    Joystick stick = new Joystick(2);
    Joystick paddleStick = new Joystick(1);

    Victor frontRight = new Victor(2) ; //7 //2
    Victor backRight = new Victor(10) ; //8 //10
    Victor frontLeft = new Victor(9) ; //9 //9
    Victor backLeft = new Victor(1) ; //10 //1
    Victor windowLeft = new Victor(5) ;
    Victor windowRight = new Victor(4) ;
    Victor kicker = new Victor (3);
    Victor hoist = new Victor (7); //1 //7
    Servo hanger = new Servo (8); //2 //8

    Encoder encoder1 = new Encoder(1, 2);
    Encoder encoder2 = new Encoder(3, 4);
    Encoder encoder3 = new Encoder(5, 6);
    Encoder encoder4 = new Encoder(7, 8);
  //  AxisCamera cam;

    AnalogChannel magnetic1 = new AnalogChannel(1);
    AnalogChannel magnetic2 = new AnalogChannel(2);

//    Timer timer1 = new Timer();

    public RobotTemplate()
    {
//        cam = AxisCamera.getInstance();
    }

    public void autonomous()
    {
        while(isAutonomous()){
            this.getWatchdog().setEnabled(false); //turn off watchdog

            /* Autonomous:
             * Make sure servos are powered to hold down hanger
             * Go forward using ServiceDriveTrain function
             * turn on kicker simultaneously
             * stop after a few seconds
             */

//            Timer.delay(8.0);
            hanger.set(1.0);
            kicker.set(0.7);
            Timer.delay(7.0); //1.0

            ServiceDriveTrain(0, -0.6, 0, 1);

            kicker.set(0.7);
            Timer.delay(3.3);
            
            frontRight.set(0); //-1 tr
            backRight.set(0); //tr
            frontLeft.set(0); // tl
            backLeft.set(0); // -1 TL
            
            kicker.set(0);
            Timer.delay(1.0);

            break;
        }
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl()
    {
        double y;
        double z;
        double x;

        // acquire all joystick values here and pass them in once needed
        // this prevents the control loop from wasting time gathering values
        while(isOperatorControl())
        {
            this.getWatchdog().setEnabled(false);

            x = stick.getX();
            y = stick.getY();
            z = stick.getZ();

            ServiceDriveTrain (x, y, z, 0);
            ServiceKicker();
        }
    }

    double fl = 0;
    double fr = 0;
    double bl = 0;
    double br = 0;

    double [] wheelSpeed = new double [4];
    double [] wheelEncoder = new double [4];
    double [] [] referenceTable = new double [5][21];

    int referenceFlag = 0;

    // use public fields as static variables to hold values for reference table

    public void matchWheelSpeeds(double _joystickSpeed)
    {
    
        /* This function matches the wheel speeds using a reference table
         * Wheel speeds are acquired from encoder and then projected to target speed
         */

        double maxError = 7;
        double encoder_error = 0;
        int referenceSpeed =  (int)(_joystickSpeed * 10) + 10;

        /* Set possible wheel speed error margin
         * Joysticks have been banded, with 10 positions so setting a value to
         * reference speed is necessary to scale the values back
         */

        // Search reference table for previous iteration's values to allow any changes to wheel speed

        if(referenceFlag == 0)
        {
            for(int i = 0; i < 5; i++)
            {
                for (int k = 0; k < 21; k++)
                {
                    referenceTable[i][k] = (k - 10.0) * 0.10;
                }
            }
            referenceFlag = 1;
        }

        //speeds will automatically be 1 if joystick moves far enough

        if(_joystickSpeed < - 0.85)
            _joystickSpeed = -1.0;

        // set each wheel to values held in field indicating previous iteration's outputted speed
        wheelSpeed [0] = fl;
        wheelSpeed [1] = fr;
        wheelSpeed [2] = bl;
        wheelSpeed [3] = br;
        
        //searched for slowest wheel

        int slowest_wheel = 0;

        for (int i = 1; i < 4; i++)
        {
            if (Math.abs(wheelEncoder[i]) < Math.abs(wheelEncoder[i - 1]))
                slowest_wheel = i;
        }

        // scaled wheel speed matching reference table's optimal values to projected speed
        for (int i = 0; i < 4; i++)
        {
            encoder_error = Math.abs(wheelEncoder[i])
                    - Math.abs(wheelEncoder[slowest_wheel]) ;

            if(encoder_error > maxError)
                referenceTable[i][referenceSpeed] /= 1.005;
            else
                referenceTable[i][referenceSpeed] *= 1.005;

            wheelSpeed [i] = referenceTable [i][referenceSpeed];

            if(wheelSpeed [i] < referenceTable [4][referenceSpeed] * 0.8)
                wheelSpeed [i] = 0.8 * referenceTable [4][referenceSpeed];

            wheelSpeed[slowest_wheel] = limit(-1, 1, wheelSpeed[slowest_wheel]);
            wheelSpeed[i] = limit(-1, 1, wheelSpeed[i]);
        }

        for(int i = 0; i < 4; i++)
        {
            if(Math.abs(wheelEncoder[i]) < 10)
                wheelSpeed[i] = _joystickSpeed;
        }
        //if wheel is too slow, then set it to at least joystick speed

        fl = wheelSpeed[0] ;
        fr = wheelSpeed[1] ;
        bl = wheelSpeed[2] ;
        br = wheelSpeed[3] ;
        // output wheel speed values
    }

    public double getAveragePosition(int _encoder)
    {
        /* our own smampling routine for the magnetic encoders
         * we grab 50 iterations woth of samples and average them out
         */

            int count = 50;
            double mr = 0;
            double ml = 0;

            double [] lencoderArray = new double [count];
            double [] rencoderArray = new double [count];

            for (int i = 0; i < count; i++)
            {
                rencoderArray [i] = magnetic1.getVoltage();
                lencoderArray [i] = magnetic2.getVoltage();
            }

            for (int i = 0; i < count; i++)
            {
                mr += rencoderArray[i];
                ml += lencoderArray[i];
            }
            //encoders

            mr /= count;
            ml /= count;

            if(_encoder == 1)
                return ml;
            else
                return mr;
    }

    double old_y;
    int joy_flag = 0;

    //joystick flags to fix position for matching wheel speeds

    public void ServiceDriveTrain (double _x, double _y, double _z, int _kill)
    {
        // joystick values passed in
        double y = -1 * _y; // flipped y due to werid joystick
        double z = _z;
        double x = _x;

        // band joystick on increments of 0.1
        y = ((int) (y * 10.0)) * 0.10;

        //create deadzones

        if (x < 0.15 && x > -0.15)
            x = 0;
        if (y < 0.2 && y > -0.2)// -0.35
            y = 0;
        if (z < 0.15 && z > -0.15)
            z = 0;

        // lock spin to prevent human error
        if(!(stick.getRawButton(2)))
            z = 0;

        /* get the magnitude of the line formed between origin and x,y
         * via the pythagorean theorem
         * this is necessary to determine wheel speeds
         * +z and -z determine the difference between either side
         * this difference allows for spins in tank drive
         */

        double tl = Math.sqrt(x * x + y * y) + z; //left wheel speed
        double tr = Math.sqrt(x * x + y * y) - z; //right wheel speed

        double mag; // what we need to get the encoders to based on joysticks
        double w; //window motor angle in radians???

        double mr = getAveragePosition(2); //right magnetic encoder from our sampling function
        double ml = getAveragePosition(1); //left magnetic encoder from our sampling function

        double tlError; // difference between left window position and target
        double trError; // difference between right window position and target

        int windowKill = _kill; //kill window motors if this has a particular value

        // kill window motors if they turned around too far
        if(mr < 0.5 || mr > 4.5 || ml < 0.5 || ml > 4.5)
            windowKill = 1;

        int trig = 0;

        // flip controls to switch front and back if button 12 is pushed
        if(stick.getRawButton(12))
        {
            y *= -1;
            x *= -1;
        }

        // hold old joystick value for wheel speeds
        if(joy_flag == 0)
        {
            old_y = y;
            joy_flag = 1;
        }
        if ((old_y - y) < 0.05 && (old_y - y) > -0.05)
            joy_flag = 1;
        else
            joy_flag = 0;

        /* flip x and y to inverse direction of robot when its going backwards
         * in a 360 degree circle the window motors can move to the angle between the (x,y)
         * coordinates and the origin but we only have a 180 degree circle
         * flipping x and y lets (-1,-1) and (1,1) to be seen as equivalent points, so the
         * robot can go to (1, 1) instead of (-1, -1) and just have the CIMs powered in the opposite direction
         */

        if(y < 0)
        {
            tl *= -1;
            tr *= -1;
            y *= -1;
            x *= -1;
        }

        // find the angle between the joystick and origin- this allows to "point and go"
        // learn trig to understand the math - it's a conversion between radians and a circle from 0 to 2

        w = MathUtils.atan2(y, x)/(2 * Math.PI) * 4;

        // (0, 0) causes math to break for atan since there is no line. so we set the wheels straight in this case
        if((x == 0) && (y == 0))
                w = 1;

        // squaring joysticks to desensitize them but keeping the sign (direction)
        if(tl < 0 && tl >= -0.9)                
            tl = -1 * tl * tl;                 
        else if (tl > 0 && tl <= 0.9)
            tl = tl * tl;

        if(tr < 0 && tr >= -0.9)
            tr = -1 * tr * tr;
        else if (tr > 0 && tr <= 0.9)
            tr = tr * tr;

        // cap joystick speeds since matt henry is always DUI
        tl = limit(-0.8, 0.8, tl);
        tr = limit(-0.8, 0.8, tr);

        // convert our arbitrary circle from 0 to 2 to an encoder position (1.25 to 3.75)
        mag= w * 1.25 + 1.25;

        //rotates everything by 90 degrees to drive with the right side as the front

        if (stick.getTrigger())
        {
            mag = w * 1.25 + 2.5;
            trig = 1;
        }

        //caps encoder values to prevent motors from overshooting
        mag = limit(1.25, 3.85, mag);

        tlError = ml - mag; // error between left window motor and target position
        trError = mr - mag; // error between right window motor and target position

        // if flag is not set do not change wheel speeds
        if(joy_flag == 0)
        {
            fl = tl;
            fr = tr;
            bl = tl;
            br = tr;
        }
       // if wheels are relatively similar in target speed match wheel speeds

        // match for sideways mode
        else if(((tl - tr) < 0.05) && ((tl - tr) > -0.05) && joy_flag == 1 && trig == 1)
        {
            matchWheelSpeeds(tl);
            frontRight.set(fr * -1 );
            backRight.set(br);
            frontLeft.set(fl);
            backLeft.set(bl * -1);
        }
        // match for normal driving direction
        else if(((tl - tr) < 0.05) && ((tl - tr) > -0.05) && joy_flag == 1 && trig == 0)
        {
            matchWheelSpeeds(tl);

            frontRight.set(fr);
            backRight.set(br * -1);
            frontLeft.set(fl * -1);
            backLeft.set(bl);
        }

        // if spinning or something with wildly varying speeds then do not match
        else
        {
            if(trig == 0)
            {
                frontRight.set(tr); //-1 tr
                backRight.set(-1* tr); //tr
                frontLeft.set(-1* tl); // tl
                backLeft.set(tl); // -1 TL
            }
            else //if(stick.getTrigger())
            {
                frontRight.set(-1 * tr);
                backRight.set(tl);
                frontLeft.set(tr);
                backLeft.set(-1 * tl);
            }
        }

        // manually set window motors
        if(stick.getRawButton(6))
        {
            windowLeft.set(stick.getX());
            windowRight.set(stick.getX());
        }
        if(stick.getRawButton(2))
        {
            windowLeft.set(0);
            windowRight.set(0);
        }
        // sample wheel encoders for the matchWheelSpeeds function
        else
        {
            encoder1.start();
            encoder2.start();
            encoder3.start();
            encoder4.start();

            if(windowKill == 0)
                windowMotorCorrect(tlError, trError, mag);

            wheelEncoder[0] = encoder2.get(); //fl
            wheelEncoder[1] = encoder4.get(); //fr
            wheelEncoder[2] = encoder3.get(); //bl
            wheelEncoder[3] = encoder1.get(); //br

            encoder1.reset();
            encoder2.reset();
            encoder3.reset();
            encoder4.reset();
        }

 //   System.out.println("mr: " + mr + " ml: " + ml);
   }

    public void windowMotorCorrect(double _tlError, double _trError, double _mag)
    {
        /* the window motors must be pulsed in order to accurately reach their position
         * otherwise they move too quickly and overshoot, forcing them to oscillate
         * we create 3 bands, one at full speed, one a slightly lower speed, and one of actualy pulses
         * in these bands the left and right side operarte in a 20 millisecond window
         * they then check their position to ensure that they have reached the target
         */

        double tlError = _tlError; // left encoder error -- see service DT function
        double trError = _trError; // right encoder error -- see service DT function

        double maxError = 0.03; // allowable error in position ~5 degrees
        double shutterError = 0.20; // size of band to shutter in
        double decelError = 0.4; // size of band to quickly decelerate
        double quickDecelError = shutterError + 0.1; // // size of band to slowly decelerate

        double wl = 0; //power of left window motor
        double wr = 0; // power of right window motor
        double shutter_speed = 1.0; //speed of motor in shutter band
        double full_speed = 1.0; // speed for fastest change
        double decel_speed = 0.5; // speed in longer band

        double move_delay = 0.02; //time the window motors run
        double brake_delay = 0.002; // length of reverse pulse on window motors
        double rev_pulse = -0.8; // power of reverse pulse

        int ldisable_flag = 1; //check if side is disabled -- TESTING code
        int rdisable_flag = 1;

        int lshutter_flag = 1; // flag if in shutter band
        int rshutter_flag = 1;

        int laccel_flag = 0; //flag if in acceleration band
        int raccel_flag = 0;

        int ldir = 1; // indicates direction of motor so it knows where to go
        int rdir = 1;

        /* the motors run in the following manner
         * motor runs at full speed in the 1st level band
         * they cut the speed at the 2nd level band
         * they then start to shutter
         * when shuttering, a forward pulse is run for a short period, then a reverse pulse
         * to drain any momentum from the signal. then the motors run at 0 to coast forward
         * then the motors are slightly powered tot build up energy for next iteration
         * once the motors are in the max error band, they are stopped
         */

        if(ldisable_flag == 1)
        {
            //start motor at full speed
            wl = 1.0;

            // if it is outside the deceleration band - in 1st band- on the left go at full speed
            if(tlError > decelError)
            {
                wl = -1 * full_speed;
                ldir = -1;
                laccel_flag = 1;
            }
            // if it's outside on the right side go full speed in opposite direction
            else if(tlError < -1 * decelError)
            {
                wl = full_speed;
                ldir = 1;
                laccel_flag = 1;
            }

            // if outside shutter band - in the 2nd band - then cut speed a little bit
            else if(tlError > shutterError)
            {
                wl = -1 * decel_speed;
                ldir = -1;
                laccel_flag = 1;
            }
            // same if outside in other side, just go in opposite direction
            else if(tlError < -1 * shutterError)
            {
                wl = decel_speed;
                ldir = 1;
                laccel_flag = 1;
            }
            // if in shutter band then cut speed some more
            else if((tlError <= shutterError) &&
                (tlError >= -1 * shutterError))
            {
                lshutter_flag = 0;
                wl = shutter_speed;

                // if overshot target zone then indicate change in dir
                if(tlError >= maxError)
                    ldir = -1;
                if(tlError <= -1 * maxError)
                    ldir = 1;

                //if in target zone then stop
                if ((tlError <= maxError) && (tlError >= -1 * maxError))
                {
                    lshutter_flag = 1; //no shutter
                    wl = 0.0;
                }
                // note direction changes
                wl *= ldir;
            }
        }
        // same as above
        if(rdisable_flag == 1)
        {
            wr = 1.0;

            if(trError > decelError)
            {
                wr = -1 * full_speed;
                rdir = -1;
                raccel_flag = 1;
            }
            else if(trError < -1 * decelError)
            {
                wr = full_speed;
                rdir = 1;
                raccel_flag = 1;
            }

            else if(trError > shutterError)
            {
                wr = -1 * decel_speed;
                rdir = -1;
                raccel_flag = 1;
            }
            else if(trError < -1 * shutterError)
            {
                wr = decel_speed;
                rdir = 1;
                raccel_flag = 1;
            }
            else if((trError <= shutterError) &&
                (trError >= -1 * shutterError))
            {
                rshutter_flag = 0;
                wr = shutter_speed;

                if(trError >= maxError)
                    rdir = -1;
                if(trError <= -1 * maxError)
                    rdir = 1;

                if ((trError <= maxError) && (trError >= -1 * maxError))
                {
                    rshutter_flag = 1; //no shutter
                    wr = 0.0;
                }
                wr *= rdir;
            }
        }

        // if shuttering then pulse the motors off
        if(lshutter_flag == 0)
            windowLeft.set(0);
        else
            windowLeft.set(wl);
        if(rshutter_flag == 0)
            windowRight.set(0);
        else
            windowRight.set(wr);

        Timer.delay(0.02); //0.05

        // then pulse on
        windowLeft.set(wl);
        windowRight.set(wr);

        // make quick positional check to stop early if possible
        double l = (magnetic2.getVoltage() - _mag);
        double r = (magnetic2.getVoltage() - _mag);

        // if in maxError then stop
        if((l >= -1 * quickDecelError) && ( l <= quickDecelError)
                && (laccel_flag == 1))
            windowLeft.set(0);
        if((r >= -1 * quickDecelError) && (r <= quickDecelError)
                && (raccel_flag == 1))
            windowRight.set(0);

        Timer.delay(move_delay);

        //quickly run a reverse pulse to pull out all momentum
        if(lshutter_flag == 0)
            windowLeft.set(rev_pulse * ldir);
        if(rshutter_flag == 0)
            windowRight.set(rev_pulse * rdir);

        Timer.delay(brake_delay);

        // then turn off motor to allow it to coast
        if(lshutter_flag == 0)
            windowLeft.set(0);
        if(rshutter_flag == 0)
            windowRight.set(0);

//        Timer.delay(0.05);

        // slightly power motors to grant it some momentum for next iteration
        if(laccel_flag == 1)
            windowLeft.set(ldir * 0.25);
        else
            windowLeft.set(0);

        if(raccel_flag == 1)
            windowRight.set(rdir * 0.25);
        else
            windowRight.set(0);
    }

    int roller_flag = 0;
    int kicker_flag = 0;

    public void ServiceKicker ()
    {
        //8 & 9 hanger
        //3 & 5 roll in/roll out
        //joystick y, kicker

        // power kicker
        if (paddleStick.getTrigger()==true){
            double flipper = paddleStick.getY();
            flipper *= -1;

            flipper = limit(-1.0, 1.0, flipper);
            
            kicker.set(flipper);
        }
        else{
            kicker.set(paddleStick.getY());
        }
        //End kicker code

        //power lift code -- hanger is servo holding tower in place
        if (paddleStick.getRawButton(8)) { //&& paddleStick.getRawButton(9)){
            hanger.set(1.0);
          //  flag = 1;
        }
        if (paddleStick.getRawButton(9)) { //&& paddleStick.getRawButton(9)){
            hanger.set(0);
        }
        else
            hanger.set(1.0);

        // turn of kicker when lift is rurnning
        if (paddleStick.getRawButton(2)){
            hoist.set(paddleStick.getY());
            kicker.set(0);            //            hoist.set(0.3);
        }
        else
            hoist.set(0);

//        System.out.println("get: " + hanger.get());
  }

    // limit value function
    public double limit(double min, double max, double input)
    {
      if (input < min)
          input = min;
      if (input > max)
          input = max;

      return input;
    }
}