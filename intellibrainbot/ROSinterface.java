import javax.comm.*;
import java.io.*;
import com.ridgesoft.intellibrain.*;
import com.ridgesoft.io.*;
import com.ridgesoft.robotics.*;
import com.ridgesoft.robotics.sensors.*;


/*
Class for communication with ROS node through serial (115.2K/8/1/N).

Accepts the following commands :
L <text to show on display> : LOG text. max 16 chars
R <sensorId1> <sensorId2> ...: READ sensorIdX. L-left IR, R-rightIR, C-sonar, O-odometry, o-plain odometry>
T <linear> <angular>: TWIST command. Given in mm and mrad/s
D <left> <right>: DRIVE command. Given in motor commands (-16..+16)

Outputs the following data:
R sensorId1:sensorData1  ...: READ sensors.  L,R & C return distance in mm;
                                             O returns x,y,heading,linear,angular in mm,mm, mrad, mm/s, mrad/s
                                             o return lef,right,lSpeed,rSpeed in ticks and motor commands
 */
public class ROSinterface {
    static String oldStr;
    static Display d;
    static PushButton startButton ;
    static DifferentialDriveController drive;
    static Sensor sensors[];

    public static void display(int line, String str)
    {
        d.print(line, str);
    }

    static void log(String str){
        if (oldStr != null)
            display(0, oldStr);
        display(1, str);
        oldStr = str;
    }

    static {
        d = IntelliBrain.getLcdDisplay();
        startButton = IntelliBrain.getStartButton();
        IntelliBrain.setTerminateOnStop(true);
        drive = new DifferentialDriveController();
        sensors = new Sensor[]{
                new IR("L",1),
                new IR("R",2),
                new Sonar("C",3),
                new Odometry("O", drive),
                new PlainOdometry("o", drive)
        };
    }

    SerialPort com;

    public ROSinterface(SerialPort com) throws Exception {
        this.com = com;
        log(com.getName());//+",push START");
        com.setSerialPortParams(115200, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
        log("115.2K/8/1/N");
    }

    void processCommand(byte[] bufferCmd, int len){
        switch (bufferCmd[0])
        {
            case 'L':
                log(new String (bufferCmd, 2, len-2));
                break;
            case 'D':
            {
                int first=2,last;
                for (last = first;last<len && bufferCmd[last]!=' ';last++){}
                float l = parseNum(bufferCmd, first, last-1);
                first = last+1;
                for (last = first;last<len && bufferCmd[last]!=' ';last++){}
                float r = parseNum(bufferCmd, first, last-1);
                if (Float.isNaN(l) || Float.isNaN(r)){
                    l = r = 0;
                }
                drive.SetSpeed((int)l,(int)r);
                break;
            }
            case 'T':
            {
                int first=2,last;
                for (last = first;last<len && bufferCmd[last]!=' ';last++){}
                float linear = parseNum(bufferCmd, first, last-1);
                first = last+1;
                for (last = first;last<len && bufferCmd[last]!=' ';last++){}
                float angular = parseNum(bufferCmd, first, last-1);
                if (Float.isNaN(linear) || Float.isNaN(angular)){
                    linear = angular = 0;
                }
                drive.SetTwist((int)linear, (int)angular);
                break;
            }
            case 'r':
            case 'R':
                byte[] buff = new byte[10];
                int p = 0;
                boolean diff;
                //log(new String (bufferCmd, 2, len-2));

                for (int ii=2;ii<=len;ii++)
                {
                    //log("TST "+ii+" "+p+new String(bufferCmd, ii, 1));
                    if (ii==len || bufferCmd[ii]==' ') {
                        //int hash = sb.toString().hashCode();
                        //log("Req "+sb.toString());
                        for (int jj=0; jj<sensors.length;jj++){
                            if (sensors[jj].id.length==p){
                                diff = false;
                                for (int kk=0;kk<p;kk++)
                                    if (sensors[jj].id[kk]!=buff[kk])
                                        diff = true;
                                if (!diff){
                                    sensors[jj].requested = bufferCmd[0]=='R';
                                    //log("Req "+new String(buff, 0, p));
                                    if (sensors[jj].requested)
                                        sensors[jj].ping();
                                    break;
                                }
                            }
                        }
                        p = 0;
                    }
                    else if (ii<len)
                    {
                        buff[p]= bufferCmd[ii];
                        p++;
                        if (p>=buff.length)
                            p=buff.length-1;
                    }
                }
                break;
        }
    }

    public void run() throws Exception {

        InputStream is = this.com.getInputStream();
        OutputStream os = this.com.getOutputStream();

        byte c=0;
        boolean run = true;

        byte buffer[] = new byte[512];
        int pos = 0, p = 0;
        byte bufferCmd[] = new byte[512];

        while (run){
            c=0;
            long t1 = System.currentTimeMillis();
            while (is.available()>0 && c!=10 && c!=13 && p<bufferCmd.length)
            {
                c = (byte)is.read();
                if (c!=10 && c!=13) {
                    bufferCmd[p]=c;
                    p++;
                }
            }
            long t2 = System.currentTimeMillis();
            if (c==10 || c==13){
                c = 0;
                if (p>0){
                    //process new command
                    if (bufferCmd[1]==' ')
                    {
                        processCommand(bufferCmd, p);
                    }
                    else
                        log("Wrong cmd:"+p);
                }
                else
                    log("Empty command");
                p = 0;
            }
            else if (p>0)
                log("Partial cmd:"+p);

            long t3 = System.currentTimeMillis();
            //long rds = 0;

            pos = 0;
            c=0;
            for (int jj=0; jj<sensors.length;jj++)
                if (sensors[jj].requested){
                    sensors[jj].ping();
                    c++;
                }

                try {
                    if (c>0)
                    Thread.sleep(25);
                }
                catch(Throwable t)
                {}



            for (int jj=0; jj<sensors.length;jj++)
                if (sensors[jj].requested){

                    if (pos==0){
                        buffer[pos] = 'R';pos++;
                    }

                    buffer[pos] = ' ';pos++;
                    pos+= sensors[jj].setID(buffer, pos);
                    buffer[pos] = ':';pos++;
                    //long tt = System.currentTimeMillis();
                    pos+= sensors[jj].setReading(buffer, pos);
                    //rds+=System.currentTimeMillis()-tt;
                    //sensors[jj].requested = false;
                }
            long t4 = System.currentTimeMillis();

            if (pos>0)
            {
                buffer[pos] = '\r';pos++;
                buffer[pos] = '\n';pos++;
                os.write(buffer, 0, pos);
                long t5 = System.currentTimeMillis();

                //log("1:"+(t5-t1)+","+(t2-t1)+","+(t3-t2));
                //log("2:"+(t4-t3)+","+(t5-t4));
            }
        }
    }

    public static void main(String args[]) {
        try {
            if (false){
                log("Press START :)");
                startButton.waitReleased();
                while(!startButton.isPressed()) {
                    Thread.sleep(10);
                }
            }

            ROSinterface iface = new ROSinterface(IntelliBrain.getCom1());
            iface.run();
        } catch (Exception ex) {
            log("Ex:" + ex.getMessage());
        }
    }

    static float parseNum(byte[] arr, int first, int last)
    {
        //log("f="+first+"l="+last);
        int len = last-first+1;
        float r = 0, s=1;
        byte c;
        int t;

        for (int ii=0; ii< len; ii++)
        {
            c = arr[last-ii];
            if (c=='-' || c=='+')
                s = c=='-'?-1:1;
            else if (c>=48 && c<=57)
            {
                t  = c-48;
                for (int jj=0; jj<ii; jj++)
                    t*=10;
                r+=t;
            }
            else{
                log(new String(arr, first, len)+"["+(last-ii)+"]"+c+"!");
                return Float.NaN;
            }

        }

        r*=s;

        //log(str+"->"+r);
        return r;
    }

    static int writeInt(byte[] buffer, int pos, int value)
    {
        return writeInt(buffer, pos, value, false);
    }
    static int writeInt(byte[] buffer, int pos, int value, boolean hex)
    {
        /*
        char[] v = Integer.toString(value).toCharArray();
        for(int ii=0;ii<v.length;ii++)
            buffer[pos+ii] = (byte) v[ii];
        return v.length;
        */
        int v = value;
        int ii = pos;
        if (value<0){
            buffer[ii]='-';ii++;
            value = -value;
        }
        boolean out = false;
        int b = //hex?268435456:
                1000000000;

        if (value<10)
            b=1;
        else if (value<100)
            b=10;
        else if (value<1000)
            b=100;
        else if (value<10000)
            b=1000;
        else if (value<100000)
            b=10000;
        else if (value<1000000)
            b=100000;

        int mod;
        while (b>0){
            if (out || value>=b){
                mod=value/b;
                if (!out && mod!=0)
                    out = true;
                if (out){
                    buffer[ii]=(byte)(mod+48);
                    ii++;
                }
                value %=b;
            }
            b/=//hex?16:
                10;
        }

        if (!out){
            buffer[ii]='0';
            ii++;
        }

        //String s = new String(buffer,pos,ii-pos);
        //ROSinterface.log(v+"->"+s+", "+(ii-pos));

        return ii-pos;
    }
}

class Sensor{
    public boolean requested = false;
    protected int hash;
    protected byte[] id;

    public int hashCode() {
        return hash;
    }

    protected Sensor(String id){
        this.id = id.getBytes();
        this.hash = id.hashCode();
    }

    int setID(byte[] buffer, int pos) {
        for (int ii=0;ii<id.length;ii++)
            buffer[pos+ii] = id[ii];
        return id.length;
    }

    int setReading(byte[] buffer, int pos) {return 0;}

    void ping(){}
}

class Ranger extends Sensor{
    protected RangeFinder rf;

    protected Ranger(String id) {
        super(id);
    }

    public int setReading(byte[] buffer, int pos) {
        int dist = (int)(rf.getDistanceCm()*10f);
        return ROSinterface.writeInt(buffer, pos, dist);
    }

    void ping() {
        rf.ping();
    }
}

class IR extends Ranger{
    public IR(String id, int port) {
        super(id);
        rf = new SharpGP2D12(IntelliBrain.getAnalogInput(port), null);
        rf.ping();
    }
}

class Sonar extends Ranger{
    public Sonar(String id, int port) {
        super(id);
        rf = new ParallaxPing(IntelliBrain.getDigitalIO(port));
        rf.ping();
    }
}

class Odometry extends Sensor{
    protected DifferentialDriveController drive;

    public Odometry(String id, DifferentialDriveController drive) {
        super(id);
        this.drive = drive;
    }

    public int setReading(byte[] buffer, int pos) {
        int oldPos = pos;

        long start = System.currentTimeMillis();
        Pose pose = drive.getPose();
        int[] twist = drive.getTwist();
        long twist_ = System.currentTimeMillis();
        int x = (int)(pose.x*1000f);
        int y = (int)(pose.y*1000f);
        int h = (int)(pose.heading*1000f);
        long data = System.currentTimeMillis();
        pos+=ROSinterface.writeInt(buffer, pos, x);
        buffer[pos]=',';pos++;
        pos+=ROSinterface.writeInt(buffer, pos, y);
        buffer[pos]=',';pos++;
        pos+=ROSinterface.writeInt(buffer, pos, h);
        buffer[pos]=',';pos++;
        long arr1 = System.currentTimeMillis();
        pos+=ROSinterface.writeInt(buffer, pos, twist[0]);
        buffer[pos]=',';pos++;
        pos+=ROSinterface.writeInt(buffer, pos, twist[1]);
        long stop = System.currentTimeMillis();

        //ROSinterface.log("1:"+(stop-start)+","+(twist_-start)+","+(data-twist_));
        //ROSinterface.log("2:"+(arr1-data)+","+(stop-arr1));


        start = System.currentTimeMillis();
        int v = 1;
        for(int ii=0;ii<30;ii++)
            v*=2;
        data = System.currentTimeMillis();
        for(int ii=0;ii<30;ii++)
            v/=2;
        stop = System.currentTimeMillis();
        //ROSinterface.log("*:"+(data-start)+", /"+(stop-data));

        return pos-oldPos;
    }

    void ping() {
    }
}

class PlainOdometry extends Sensor{
    protected DifferentialDriveController drive;

    public PlainOdometry(String id, DifferentialDriveController drive) {
        super(id);
        this.drive = drive;
    }

    public int setReading(byte[] buffer, int pos) {
        int oldPos = pos;

        int[] counts = drive.getCounts();
        int[] speed = drive.getSpeed();
        pos+=ROSinterface.writeInt(buffer, pos, counts[0]);
        buffer[pos]=',';pos++;
        pos+=ROSinterface.writeInt(buffer, pos, counts[1]);
        buffer[pos]=',';pos++;
        pos+=ROSinterface.writeInt(buffer, pos, speed[0]);
        buffer[pos]=',';pos++;
        pos+=ROSinterface.writeInt(buffer, pos, speed[1]);

        return pos-oldPos;
    }

    void ping() {
    }
}


class DifferentialDriveController {
    private static Motor leftMotor, rightMotor;
    private static AnalogInput leftWheelInput, rightWheelInput;
    private static ShaftEncoder leftEncoder, rightEncoder;
    static Localizer location;

    static float wheelDiameter = 0.065f, trackWidth = 0.117f;
    static int countsPerRevolution = 16;

    static final int localizerThreadPriority = Thread.MAX_PRIORITY-1, localizerPeriod = 30;
    static final int MAX_SPEED = 16;

    int lastLeft = 0;
    int lastRight = 0;

    float lengthPerRotation, speedUnit, maxTranSpeed, maxRotSpeed,rotUnit;

    public DifferentialDriveController () {
        leftWheelInput = IntelliBrain.getAnalogInput(4);
        rightWheelInput = IntelliBrain.getAnalogInput(5);

        // Create wheel (shaft) encoders from your robot's infrared wheel sensors.
        leftEncoder = new AnalogShaftEncoder(leftWheelInput, 250, 750, localizerPeriod, Thread.MAX_PRIORITY);
        rightEncoder = new AnalogShaftEncoder(rightWheelInput, 250, 750, localizerPeriod, Thread.MAX_PRIORITY);

        leftMotor = new ContinuousRotationServo(IntelliBrain.getServo(1), false,
                14, (DirectionListener) leftEncoder);
        rightMotor = new ContinuousRotationServo(IntelliBrain.getServo(2), true,
                14, (DirectionListener) rightEncoder);

        location = new OdometricLocalizer(leftEncoder, rightEncoder, wheelDiameter,
                trackWidth, countsPerRevolution, localizerThreadPriority, localizerPeriod);

        lengthPerRotation = (float)(Math.PI * wheelDiameter);
        maxTranSpeed = (50f/60f) * lengthPerRotation;//50RPM@16 = 5/6 rot/per second @ 16
        speedUnit = maxTranSpeed/MAX_SPEED;
        maxRotSpeed = 2f*maxTranSpeed/trackWidth    ;//2pi RAD * (speed/pi*width) ROT/SEC
        rotUnit = maxRotSpeed/(MAX_SPEED*2f);
    }

    public int[] getCounts()
    {
        return new int[]{ leftEncoder.getCounts(), rightEncoder.getCounts() };
    }

    public void SetTwist(int linear1000, int angular1000) {
        int l, r;
        l = r = (int) ((linear1000/1000f)/speedUnit);
        int diff = (int) ((angular1000/1000f)/rotUnit);
        l-=diff/2;
        r+=diff/2+diff % 2;

        //ROSinterface.log(linear1000+","+angular1000+"->"+l+","+r);

        SetSpeed(l,r);
    }

    public int[] getTwist() {
        int linear1000 = (int)(1000f*speedUnit*(lastLeft+lastRight)/2f);

        int angular1000 = (int)(1000f*rotUnit*(lastRight-lastLeft));

        return new int[]{linear1000, angular1000};
    }

    public int[] getSpeed() {
        return new int[]{lastLeft, lastRight};
    }

    public void SetSpeed(int l, int r) {
        if (Math.abs(l)>MAX_SPEED || Math.abs(r)>MAX_SPEED){
            float c = (1f*MAX_SPEED)/Math.max(Math.abs(l),Math.abs(r));
            l = (l>=0?1:-1)*Math.round(Math.abs(l*c));
            r = (r>=0?1:-1)*Math.round(Math.abs(r*c));
        }

        leftMotor.setPower(l);
        rightMotor.setPower(r);
        lastLeft = l;
        lastRight = r;
        //ROSinterface.log("Speed="+l+","+r);
    }

    public Pose getPose()
    {
        return location.getPose();
    }
}
