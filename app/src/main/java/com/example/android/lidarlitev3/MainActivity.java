package com.example.android.lidarlitev3;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.IOIO.VersionType;
import ioio.lib.api.PulseInput;
import ioio.lib.api.TwiMaster;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;
import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;
import android.widget.ToggleButton;


/**
 * This is the main activity of the HelloIOIO example application.
 *
 * It displays a toggle button on the screen, which enables control of the
 * on-board LED. This example shows a very simple usage of the IOIO, by using
 * the {@link IOIOActivity} class. For a more advanced use case, see the
 * HelloIOIOPower example.
 */
public class MainActivity extends IOIOActivity {
    private ToggleButton button_;

    TwiMaster twi;
    //LidarLite lidar;

    byte LIDAR_ADDRESS = 0x62;		//Default slave address
    int cal_cnt = 0;
    int dis = 0;

    //Registers
    private byte ACQ_COMMAND = 0x00;					//Receiver bias correction
    private byte[] STATUS = new byte[]{(byte) 0x01};	//System status
    private byte SIG_COUNT_VAL = (byte) 0x02;			//Measurement rate & range
    private byte ACQ_CONFIG_REG = (byte) 0x04;			//Measurement speed & accuracy
    private byte THRESHOLD_BYPASS = (byte) 0x1c;		//Detection sensitivity
    private byte FULL_DELAY_HIGH = (byte) 0x0f;			//Distance measurement high byte (cm)
    private byte FULL_DELAY_LOW = (byte) 0x10;			//Distance measurement low byte (cm)

    private byte[] measurementArray = new byte[] {FULL_DELAY_HIGH, FULL_DELAY_LOW};
    //private byte[] measurementArray = {FULL_DELAY_HIGH};

    private byte[] empy_read = new byte[] {};

    private DigitalOutput trigger_port;
    private PulseInput monitor_port;

    int trigger_pin = 38;
    int monitor_pin = 40;
    float pulseDistance;

    /**
     * Called when the activity is first created. Here we normally initialize
     * our GUI.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

    /**
     * This is the thread on which all the IOIO activity happens. It will be run
     * every time the application is resumed and aborted when it is paused. The
     * method setup() will be called right after a connection with the IOIO has
     * been established (which might happen several times!). Then, loop() will
     * be called repetitively until the IOIO gets disconnected.
     */
    class Looper extends BaseIOIOLooper {
        /**
         * Called every time a connection with IOIO has been established.
         * Typically used to open pins.
         *
         * @throws ConnectionLostException
         *             When IOIO connection is lost.
         *
         * @see ioio.lib.util.IOIOLooper#setup()
         */
        @Override
        protected void setup() throws ConnectionLostException, InterruptedException {
            showVersions(ioio_, "IOIO connected!");
//            twi = ioio_.openTwiMaster(1, TwiMaster.Rate.RATE_100KHz, false);

            trigger_port = ioio_.openDigitalOutput(trigger_pin, false);
            monitor_port = ioio_.openPulseInput(monitor_pin, PulseInput.PulseMode.POSITIVE);

            //configure for default mode, balanced performance
//            configure(LIDAR_ADDRESS,0);
            //Thread.sleep(100);
            enableUi(true);
        }

        /**
         * Called repetitively while the IOIO is connected.
         *
         * @throws ConnectionLostException
         *             When IOIO connection is lost.
         * @throws InterruptedException
         * 				When the IOIO thread has been interrupted.
         *
         * @see ioio.lib.util.IOIOLooper#loop()
         */
        @Override
        public void loop() throws ConnectionLostException, InterruptedException {
            Log.i("LIDAR","---loop start---");

            // getDuration returns pulse width in seconds; 10us = 1cm
            // getDurations [s] * (1000000 us / 1s) * (1 cm / 10 us) * (1E-3 km / 1E2 cm) = 1 km
            pulseDistance = monitor_port.getDuration();

            Log.i("DISTANCE","" + pulseDistance + " km");

/*
            if (cal_cnt == 0) {
                dis = distance(LIDAR_ADDRESS,true);	//Measurement w/ bias correction
                //Log.i("MEASURED","bias");
            } else {
                dis = distance(LIDAR_ADDRESS,false); //Measurement w/o bias correction
                //Log.i("MEASURED","no bias");
            }

            //Increment reading counter
            cal_cnt++;
            cal_cnt = cal_cnt % 10;

            //Display distance reading
            Log.i("DISTANCE","" + dis + " cm");
            Thread.sleep(10);
*/
        }

        /**
         * Called when the IOIO is disconnected.
         *
         * @see ioio.lib.util.IOIOLooper#disconnected()
         */
        @Override
        public void disconnected() {
//            twi.close();
            monitor_port.close();
            enableUi(false);
            toast("IOIO disconnected");
        }

        /**
         * Called when the IOIO is connected, but has an incompatible firmware version.
         *
         * @see ioio.lib.util.IOIOLooper#incompatible(IOIO)
         */
        @Override
        public void incompatible() {
            showVersions(ioio_, "Incompatible firmware version!");
        }
    }

    /**
     * A method to create our IOIO thread.
     *
     * @see ioio.lib.util.AbstractIOIOActivity#createIOIOThread()
     */
    @Override
    protected IOIOLooper createIOIOLooper() {
        return new Looper();
    }

    private void showVersions(IOIO ioio, String title) {
        toast(String.format("%s\n" +
                        "IOIOLib: %s\n" +
                        "Application firmware: %s\n" +
                        "Bootloader firmware: %s\n" +
                        "Hardware: %s",
                title,
                ioio.getImplVersion(VersionType.IOIOLIB_VER),
                ioio.getImplVersion(VersionType.APP_FIRMWARE_VER),
                ioio.getImplVersion(VersionType.BOOTLOADER_VER),
                ioio.getImplVersion(VersionType.HARDWARE_VER)));
    }

    private void toast(final String message) {
        final Context context = this;
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(context, message, Toast.LENGTH_LONG).show();
            }
        });
    }

    private int numConnected_ = 0;

    private void enableUi(final boolean enable) {
        // This is slightly trickier than expected to support a multi-IOIO use-case.
        runOnUiThread(new Runnable() {
            @Override
            public void run() {

            }
        });
    }

    // Takes latitude & longitude of current position (degree minutes, dm), sensor distance reading
    // (km), trueCourse (rad); Returns GPS coords as dm
    double[] calculateMannequinnGpsCoordinates(double lat_dm, double lon_dm, double distance_km, double trueCourse_rad) {
        double lat_rad = lat_dm * Math.PI / 180;
        double lon_rad = lon_dm * Math.PI / 180;
        double distance_rad = distance_km / 6371;       //6371 is the Earth's radius in km


        double lat = Math.asin(Math.sin(lat_rad)*Math.cos(distance_rad) + Math.cos(lat_rad)*Math.sin(distance_rad)*Math.cos(trueCourse_rad));
        double lon;

        if (Math.cos(lat) == 0) {
            lon = lon_rad;
        } else {
            lon = (((lon_rad - Math.asin(Math.sin(trueCourse_rad)*Math.sin(distance_rad) / Math.cos(lat))) + Math.PI) % (2*Math.PI)) - Math.PI;
        }

        lat = Math.toDegrees(lat);
        lon = Math.toDegrees(lon);

        double[] gps = {lat,lon};
        return gps;
    }

    /*------------------------------------------------------------------------------
   Configure

   Selects one of several preset configurations.

   Parameters
   ------------------------------------------------------------------------------
   configuration:  Default 0.
       0: Default mode, balanced performance.
       1: Short range, high speed. Uses 0x1d maximum acquisition count.
       2: Default range, higher speed short range. Turns on quick termination
          detection for faster measurements at short range (with decreased
          accuracy)
       3: Maximum range. Uses 0xff maximum acquisition count.
       4: High sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for high sensitivity and noise.
       5: Low sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for low sensitivity and noise.
      address: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
   ------------------------------------------------------------------------------*/
    void configure(byte address, int configuration) throws ConnectionLostException, InterruptedException {
        switch (configuration) {
            case 0: //Default mode, balanced performance
                write(address, SIG_COUNT_VAL, (byte)0x80);		//Default
                write(address, ACQ_CONFIG_REG, (byte)0x08);		//Default
                write(address, THRESHOLD_BYPASS, (byte)0x00);	//Default
                break;
            case 1: //Short range, high rate
                write(address, SIG_COUNT_VAL, (byte) 0xd1);
                write(address, ACQ_CONFIG_REG, (byte)0x08);		//Default
                write(address, THRESHOLD_BYPASS, (byte)0x00);	//Default
                break;
            case 2: //Default range, higher speed, low accuracy
                write(address, SIG_COUNT_VAL, (byte)0x80);		//Default
                write(address, ACQ_CONFIG_REG, (byte)0x00);
                write(address, THRESHOLD_BYPASS, (byte)0x00);	//Default
                break;
            case 3:	//Maximum range
                write(address, SIG_COUNT_VAL, (byte)0xff);
                write(address, ACQ_CONFIG_REG, (byte)0x08);		//Default
                write(address, THRESHOLD_BYPASS, (byte)0x00);	//Default
                break;
            case 4: //High sensitivity detection, high erroneous measurements
                write(address, SIG_COUNT_VAL, (byte)0x80);		//Default
                write(address, ACQ_CONFIG_REG, (byte)0x08);		//Default
                write(address, THRESHOLD_BYPASS, (byte)0x80);
                break;
            case 5: //Low sensitivity detection, low erroneous measurements
                write(address, SIG_COUNT_VAL, (byte)0x80);		//Default
                write(address, ACQ_CONFIG_REG, (byte)0x08);		//Default
                write(address, THRESHOLD_BYPASS, (byte)0xb0);
                break;
        }
    }   /* configure */

    /*------------------------------------------------------------------------------
     Reset

     Reset device. The device reloads default register settings, including the
     default I2C address. Re-initialization takes approximately 22ms.

     Parameters
     ------------------------------------------------------------------------------
    address: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
    ------------------------------------------------------------------------------*/
    void reset (byte address) throws ConnectionLostException, InterruptedException {
        write(address, ACQ_COMMAND, (byte)0x00);
    }   /* reset */

    /*------------------------------------------------------------------------------
    Distance

    Take a distance measurement and read the result.

    Process
    ------------------------------------------------------------------------------
    1.  Write 0x04 or 0x03 to register 0x00 to initiate an aquisition.
    2.  Read register 0x01 (this is handled in the read() command)
        - if the first bit is "1" then the sensor is busy, loop until the first
          bit is "0"
        - if the first bit is "0" then the sensor is ready
    3.  Read two bytes from register 0x8f and save
    4.  Shift the first value from 0x8f << 8 and add to second value from 0x8f.
        The result is the measured distance in centimeters.

    Parameters
    ------------------------------------------------------------------------------
    address: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
    biasCorrection: Default true. Take acquisition with receiver bias
        correction. If set to false measurements will be faster. Receiver bias
        correction must be performed periodically. (e.g. 1 out of every 100
        readings).
    ------------------------------------------------------------------------------*/
    int distance(byte address, boolean biasCorrection) throws ConnectionLostException, InterruptedException{
        byte[] distanceArray = new byte[] {(byte)0x00,(byte)0x00,(byte)0x00,(byte)0x00,(byte)0x00};		//Array to store high & low bytes of distance
        int distance;

        if (biasCorrection) {
            //Take acquisition & correlation processing with receiver bias correction
            write(address, ACQ_COMMAND, (byte)0x04);
        } else {
            //Take acquisition & correlation processing w/o receiver bias correction
            write(address, ACQ_COMMAND, (byte)0x03);
        }

        //read two bytes from 0x0f & 0x10
        read(address, true, measurementArray, distanceArray);

        //shift high byte [0] 8 to the left & add low byte [1] to create 16 bit int
        distance = ((distanceArray[0] << 8) + distanceArray[1]);

        //Log.i("MEASURED","(distanceArray[0] << 8) = " + (distanceArray[0] << 8) + "; distanceArray[1] = " + distanceArray[1]);

        return distance;
    }   /* distance */

    /*------------------------------------------------------------------------------
    Write

    Perform TWI (I2C) write to device.

    Parameters
    ------------------------------------------------------------------------------
    address: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
    register: register address to write to.
    value: value to write.
    ------------------------------------------------------------------------------*/
    private void write(byte address, byte register, byte value) throws ConnectionLostException, InterruptedException {
        byte[] writing = new byte[] {register, value};

        twi.writeRead(address, false, writing, writing.length, empy_read, 0);
        Log.i("WRITING","Completed");
        Thread.sleep(1);	//sleep 1 ms
    }   /* write */


    /*------------------------------------------------------------------------------
    Read

    Perform I2C read from device. Will detect an unresponsive device and report
    the error over serial. The optional busy flag monitoring
    can be used to read registers that are updated at the end of a distance
    measurement to obtain the new data.

    Parameters
    ------------------------------------------------------------------------------
    monitorBusyFlag: if true, the routine will repeatedly read the status
        register until the busy flag (LSB) is 0.
    register: register address to read from.
    saveData: an array to store the read values.
    ------------------------------------------------------------------------------*/
    private void read(byte address, boolean monitorBusyFlag, byte[] register, byte[] saveData) throws ConnectionLostException, InterruptedException {
        int busyFlag = 0;		//monitors when the device is done with a measurement
        int busyCounter = 0; 	//counts the # of times busy flag is checked, for timeout
        byte[] busyReading = new byte[] {(byte)0x00,(byte)0x00};
/*
		if (twi.writeRead(address, false, register, register.length, saveData, saveData.length)) {
			//twi.writeRead(address, false, register, register.length, saveData, saveData.length);

			for (byte item:saveData) {
				Log.i("READING","saveData = " + item);
			}
			Log.i("READING","success");
		} else {
			Log.i("READING","> nack");
		}
*/

        if (monitorBusyFlag) {
            busyFlag = 1; //Begin read immediately if not monitoring busy flag
        }

        while (busyFlag != 0) {	//Loop until device is not busy
            if (twi.writeRead(address, false, STATUS, 1, busyReading, busyReading.length)) {
                //twi.writeRead(address, false, STATUS, 1, busyReading, busyReading.length);

            } else {
                Log.i("READING","> nack");
            }
            Log.i("BUSY","" + busyReading[0]);
            busyFlag = bitRead(busyReading[0],0);	//Read LSB of status register for busy flag
            Log.i("BUSY","" + busyFlag);
            busyCounter++;

            //Handle timeout condition, exit while loop
            if (busyCounter > 9999) {
                busyCounter = 0;
                Log.i("READING","Failed");
                break;
            }
        }

        //Device is not busy, begin reading
        if (busyFlag == 0) {
            if (twi.writeRead(address, false, register, register.length, saveData, saveData.length)) {
                //twi.writeRead(address, false, register, register.length, saveData, saveData.length);

                for (byte item:saveData) {
                    Log.i("READING","saveData = " + item);
                }
                Log.i("READING","success");
            } else {
                Log.i("READING","> nack");
            }
        }
    }   /* read */

    //read position pos from byte aByte
    private int bitRead(byte value, int bit) {
        return (((value) >> (bit)) & 0x01);
    }   /* bitRead */

}