package com.balancerobot;

import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.PowerManager;
import android.preference.PreferenceManager;

import android.support.v7.app.ActionBarActivity;
import android.support.v7.widget.Toolbar;
import android.view.KeyEvent;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.view.inputmethod.EditorInfo;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import java.util.InputMismatchException;
import java.util.StringTokenizer;

public class MainActivity extends ActionBarActivity {

    private static final String TAG = "MainActivity";

    private Context mContext;
    private static MainActivity mContext1;
    private View menuView;


    MenuItem itemtemp;
    byte[] commandPacket = new byte[11];
    byte[] commandPacket1 = new byte[2];
    byte[] commandPacket2 = new byte[4];
    static byte[] commandPacketBlueTooth = new byte[6];

    public static int Kp = 0;
    public static int Ki = 0;
    public static int Kd = 0;

    private Button Pids;
    private Button Forward;
    private Button Back;
    private Button Left;
    private Button Right;
    private Button Stop;
    private Button C_Plus;
    private Button C_Mines;

    static TextView textView_Speed1;
    static TextView textView_Speed2;
    static TextView  textPwmL;
    static TextView  textPwmR;
    static TextView  textSpeedL;
    static TextView  textAngle;
    static TextView  textSpeedR;

    static SeekBar seekBar_Speed1;
    static SeekBar seekBar_Speed2;

    private int speed1;
    private int speed2;

    private int azimuth;
    private int pitch;
    private int roll;
    private SensorManager mSensorManager;

    public static final int MESSAGE_STATE_CHANGE = 1;
    public static final int MESSAGE_READ = 2;
    public static final int MESSAGE_WRITE = 3;
    public static final int MESSAGE_DEVICE_NAME = 4;
    public static final int MESSAGE_TOAST = 5;
    // Key names received from the BluetoothChatService Handler
    public static final String DEVICE_NAME = "device_name";
    public static final String TOAST = "toast";
    // Intent request codes
    private static final int REQUEST_CONNECT_DEVICE = 2;
    private static final int REQUEST_ENABLE_BT = 3;

    BluetoothDevice currentdevice;
    boolean commandmode = false, initialized = false, tryconnect = false;
    Intent serverIntent = null;

    private PowerManager.WakeLock wl;
    private Menu menu;
    private EditText mOutEditText;
    private TextView statusText;
    private Button mSendButton;
    private ListView mConversationView;
    private String mConnectedDeviceName = "Ecu";
    // Local Bluetooth adapter
    private BluetoothAdapter mBluetoothAdapter = null;
    // Member object for the chat services
    private static BluetoothService  mBtService = null;

    // The Handler that gets information back from the BluetoothChatService
    // Array adapter for the conversation thread
    private ArrayAdapter<String> mConversationArrayAdapter;

    private final Handler mBtHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {

            switch (msg.what) {
                case MESSAGE_STATE_CHANGE:

                    switch (msg.arg1) {
                        case BluetoothService.STATE_CONNECTED:
                            statusText.setText(getString(R.string.title_connected_to, mConnectedDeviceName));
                            try {
                                itemtemp = menu.findItem(R.id.menu_connect_bt);
                                itemtemp.setTitle(R.string.disconnectbt);
                            } catch (Exception e) {
                            }
                            break;
                        case BluetoothService.STATE_CONNECTING:
                            statusText.setText(R.string.title_connecting);
                            break;
                        case BluetoothService.STATE_LISTEN:

                        case BluetoothService.STATE_NONE:
                            statusText.setText(R.string.title_not_connected);
                            itemtemp = menu.findItem(R.id.menu_connect_bt);
                            itemtemp.setTitle(R.string.connectbt);
                            break;
                    }
                    break;
                case MESSAGE_WRITE:

                    byte[] writeBuf = (byte[]) msg.obj;
                    String writeMessage = new String(writeBuf);

                    if (commandmode || !initialized) {
                        mConversationArrayAdapter.add("Command:  " + writeMessage);
                    }

                    break;
                case MESSAGE_READ:

                    String tmpmsg = clearMsg(msg);

                    if(tmpmsg.contains("Data:"))
                    {
                        String Title = "Data";
                        String Pwml = "0";
                        String Pwmr = "0";
                        String Angle ="0";
                        String Speed_Need = "0";
                        String Turn_Need = "0";
                        String Speed_L = "0";
                        String Speed_R = "0";
                        String Temperature = "0";
                        String KP = "0";
                        String KI = "0";
                        String KD = "0";
                        String Ultarsonic_Distance = "0";
                        String Position_Add = "0";
                        String Correction = "0";

                        StringTokenizer token = new StringTokenizer(tmpmsg, ":");

                        if(token.hasMoreTokens()) {
                             Title = token.nextToken();
                        }

                        if(token.hasMoreTokens())
                        {
                             Pwml = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                             Pwmr = token.nextToken();
                        }
                        if(token.hasMoreTokens()) {
                             Angle = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                            Speed_Need = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                            Turn_Need = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                            Speed_L = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                            Speed_R = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                            KP = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                            KI = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                            KD = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                            Temperature = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                            Position_Add = token.nextToken();
                        }
                        if(token.hasMoreTokens())
                        {
                            Correction = token.nextToken();
                        }

                        try {
                            Kp = (int) Math.round(Float.valueOf(KP));
                            Ki = (int) Math.round(Float.valueOf(KI));
                            Kd = (int) Math.round(Float.valueOf(KD) * 5);
                        } catch (NumberFormatException nfe) {
                        }

                        tmpmsg =   "KP : " + KP + "  KI : " + KI + "  KD : " + KD + "\n"
                                 + "Speed_Need: : " + Speed_Need +  "  Turn_Need: " + Turn_Need + "\n"
                                 + "Temp: : " + Temperature +  "  Position: " + Position_Add +  "  Correction: " + Correction;

                        textSpeedL.setText("L: " + Speed_L);
                        textPwmR.setText("PR: " + Pwmr);
                        textPwmL.setText("PL: " + Pwml);
                        textAngle.setText("A: " + Angle);
                        textSpeedR.setText("R: " + Speed_R);

                    }

                    mConversationArrayAdapter.add(mConnectedDeviceName + ":  " + tmpmsg);

                    break;
                case MESSAGE_DEVICE_NAME:
                    // save the connected device's name
                    mConnectedDeviceName = msg.getData().getString(DEVICE_NAME);
                    break;
                case MESSAGE_TOAST:
                    Toast.makeText(getApplicationContext(), msg.getData().getString(TOAST),
                            Toast.LENGTH_SHORT).show();
                    break;
            }
        }
    };

    private String clearMsg(Message msg) {

        String tmpmsg = msg.obj.toString();
        tmpmsg = tmpmsg.replace("null", "");
        //tmpmsg = tmpmsg.replaceAll("\\s", ""); //removes all [ \t\n\x0B\f\r]
        tmpmsg = tmpmsg.replace("\n", "").replace("\r", "");
        tmpmsg = tmpmsg.replaceAll(">", "");

        return tmpmsg;
    }

    // The action listener for the EditText widget, to listen for the return key
    private TextView.OnEditorActionListener mWriteListener =
        new TextView.OnEditorActionListener() {
            public boolean onEditorAction(TextView view, int actionId, KeyEvent event) {
                // If the action is a key-up event on the return key, send the message
                if (actionId == EditorInfo.IME_NULL && event.getAction() == KeyEvent.ACTION_UP) {
                    String message = view.getText().toString();
                    sendMessage(message);
                }
                return true;
            }
        };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        mContext = this;
        mContext1 = this;

        PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
        wl = pm.newWakeLock(PowerManager.SCREEN_BRIGHT_WAKE_LOCK, "My Tag");
        wl.acquire();

        getWindow().setSoftInputMode(
                WindowManager.LayoutParams.SOFT_INPUT_STATE_ALWAYS_HIDDEN
        );

        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        /*getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);*/

        statusText = (TextView) findViewById(R.id.text_status);
        mOutEditText = (EditText) findViewById(R.id.edit_text_out);
        mSendButton = (Button) findViewById(R.id.button_send);
        mConversationView = (ListView) findViewById(R.id.in);

        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (mBluetoothAdapter == null) {
            Toast.makeText(getApplicationContext(), "Bluetooth is not available", Toast.LENGTH_LONG).show();
        }
        else
        {
            if (mBtService != null) {
                if (mBtService.getState() == BluetoothService.STATE_NONE) {
                    mBtService.start();
                }
            }
        }

        // Initialize the array adapter for the conversation thread
        mConversationArrayAdapter = new ArrayAdapter<String>(this, R.layout.message) {
            @Override
            public View getView(int position, View convertView, ViewGroup parent) {
                // Get the Item from ListView
                View view = super.getView(position, convertView, parent);

                // Initialize a TextView for ListView each Item
                TextView tv = (TextView) view.findViewById(R.id.listText);

                // Set the text color of TextView (ListView Item)
                tv.setTextColor(Color.parseColor("#FFFFFF"));
                tv.setTextSize(12);

                // Generate ListView Item using TextView
                return view;
            }
        };

        mConversationView.setAdapter(mConversationArrayAdapter);

        mSendButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Send a message using content of the edit text widget
                String message = mOutEditText.getText().toString();
                sendMessage(message);
            }
        });

        mOutEditText.setOnEditorActionListener(mWriteListener);

        textView_Speed1 = (TextView)findViewById(R.id.textView_Speed1);
        textView_Speed2 = (TextView)findViewById(R.id.textView_Speed2);
        textPwmL = (TextView)findViewById(R.id.textPwmL);
        textPwmR = (TextView)findViewById(R.id.textPwmR);
        textSpeedL = (TextView)findViewById(R.id.textSpeedL);
        textSpeedR = (TextView)findViewById(R.id.textSpeedR);
        textAngle = (TextView)findViewById(R.id.textAngle);

        seekBar_Speed1 = (SeekBar)findViewById(R.id.seekBar_Speed1);
        seekBar_Speed1.setMax(255);
        //seekBar_Speed1.setProgress(MakerStudioDemo.getKAngle());

        seekBar_Speed2 = (SeekBar) findViewById(R.id.seekBar_Speed2);
        seekBar_Speed2.setMax(255);

        SeekBar.OnSeekBarChangeListener seekBar_Speed1_Listener = new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,
                                          boolean fromUser) {
                //App.setKAngle(seekBar_KAngle.getProgress());
                textView_Speed1.setText("Forward-Reverse Speed : " + seekBar_Speed1.getProgress());
                speed1 = seekBar_Speed1.getProgress();
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                // Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                // Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
            }

        };
        seekBar_Speed1.setOnSeekBarChangeListener(seekBar_Speed1_Listener);

        SeekBar.OnSeekBarChangeListener seekBar_Speed2_Listener = new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,
                                          boolean fromUser) {
                textView_Speed2.setText("Turn Speed : " + seekBar_Speed2.getProgress());
                speed2 = seekBar_Speed2.getProgress();
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                // Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                // Toast.makeText(getApplicationContext(), "onStopTrackingTouch",Toast.LENGTH_SHORT).show();
            }

        };
        seekBar_Speed2.setOnSeekBarChangeListener(seekBar_Speed2_Listener);

        commandPacket[0] = (byte) 0xFF;
        commandPacket[1] = (byte) 0xAA;
        commandPacket[2] = 0x00;
        commandPacket[3] = 0x00;
        commandPacket[4] = 0x00;
        commandPacket[5] = 0x00;
        commandPacket[8] = 0x00;
        commandPacket[9] = 0x00;
        commandPacket[10] = 0x02;
        commandPacket1[0] = 0x30;
        commandPacket1[1] = 0x30;
        commandPacketBlueTooth[0] = (byte)0xAA;

        LayoutInflater mLayoutInflater = (LayoutInflater) getSystemService(LAYOUT_INFLATER_SERVICE);
        menuView = (View) mLayoutInflater.inflate(
                R.layout.activity_main, null, true);


        Pids = (Button) findViewById(R.id.Pids);
        Pids.setOnClickListener(VoiceClickListener);

        Forward = (Button) findViewById(R.id.Forward);
        Forward.setOnTouchListener(new Button.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                switch (action) {
                    case MotionEvent.ACTION_DOWN:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x01;
                        commandPacketBlueTooth[3] = 0x03;
                        commandPacketBlueTooth[4] = (byte)speed1;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;

                    case MotionEvent.ACTION_UP:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x00;
                        commandPacketBlueTooth[3] = 0x03;
                        commandPacketBlueTooth[4] = 0x30;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;
                }
                return false;
            }
        });


        Back = (Button) findViewById(R.id.Back);
        Back.setOnTouchListener(new Button.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                switch (action) {
                    case MotionEvent.ACTION_DOWN:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x02;
                        commandPacketBlueTooth[3] = 0x00;
                        commandPacketBlueTooth[4] = (byte)speed1;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;

                    case MotionEvent.ACTION_UP:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x00;
                        commandPacketBlueTooth[3] = 0x00;
                        commandPacketBlueTooth[4] = 0x30;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;
                }
                return false;
            }
        });

        Left = (Button) findViewById(R.id.Left);
        Left.setOnTouchListener(new Button.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                switch (action) {
                    case MotionEvent.ACTION_DOWN:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x03;
                        commandPacketBlueTooth[3] = 0x00;
                        commandPacketBlueTooth[4] = (byte)speed2;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;

                    case MotionEvent.ACTION_UP:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x00;
                        commandPacketBlueTooth[3] = 0x03;
                        commandPacketBlueTooth[4] = 0x03;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;
                }
                return false;
            }
        });

        Right = (Button) findViewById(R.id.Right);
        Right.setOnTouchListener(new Button.OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                switch (action) {
                    case MotionEvent.ACTION_DOWN:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x04;
                        commandPacketBlueTooth[3] = 0x7F;
                        commandPacketBlueTooth[4] = (byte)speed2;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;

                    case MotionEvent.ACTION_UP:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x00;
                        commandPacketBlueTooth[3] = 0x03;
                        commandPacketBlueTooth[4] = 0x03;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;
                }
                return false;
            }
        });

        Stop = (Button) findViewById(R.id.Stop);
        Stop.setOnTouchListener(new Button.OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                switch (action) {
                    case MotionEvent.ACTION_DOWN:
                        commandPacket[0] = 0x30;
                        commandPacket[1] = 0x41;
                        commandPacket[2] = 0x30;
                        commandPacket[3] = 0x32;
                        commandPacket[4] = 0x0d;
                        commandPacket[5] = 0x0a;
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacket);
                        break;

                    case MotionEvent.ACTION_UP:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x00;
                        commandPacketBlueTooth[3] = 0x03;
                        commandPacketBlueTooth[4] = 0x03;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;
                }
                return false;
            }
        });


        C_Plus = (Button) findViewById(R.id.C_Plus);
        C_Plus.setOnTouchListener(new Button.OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                switch (action) {
                    case MotionEvent.ACTION_DOWN:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x05;
                        commandPacketBlueTooth[3] = 0x00;
                        commandPacketBlueTooth[4] = (byte)speed2;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;
                }
                return false;
            }
        });

        C_Mines = (Button) findViewById(R.id.C_Mines);
        C_Mines.setOnTouchListener(new Button.OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                switch (action) {
                    case MotionEvent.ACTION_DOWN:
                        commandPacketBlueTooth[1] = 0x03;
                        commandPacketBlueTooth[2] = 0x06;
                        commandPacketBlueTooth[3] = 0x7F;
                        commandPacketBlueTooth[4] = (byte)speed2;
                        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
                        if (mBtService != null)
                            mBtService.sendCmd(commandPacketBlueTooth);
                        break;
                }
                return false;
            }
        });

    }

    private View.OnClickListener VoiceClickListener = new View.OnClickListener() {
        @Override
        public void onClick(View arg0) {
            //WIFIRobotControl.showAlgoPIDComponentDlg(mContext, menuView);
            //WIFI_Control_CommandPacket[0] = 0x0A;//喜欢
            //if (Linkstate == 1)
            //	sendCmd(WIFI_Control_CommandPacket);
            if (mBtService != null) {
                textView_Speed1.setVisibility(View.INVISIBLE);
                textView_Speed2.setVisibility(View.INVISIBLE);
                seekBar_Speed1.setVisibility(View.INVISIBLE);
                seekBar_Speed2.setVisibility(View.INVISIBLE);
                PIDSet.showPIDSetDialog(mContext, menuView);
            }
        }
    };

    private SensorEventListener mSensorEventListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent event) {

            azimuth = (int)event.values[SensorManager.DATA_X];
            pitch = (int)event.values[SensorManager.DATA_Y];
            roll = (int)event.values[SensorManager.DATA_Z];

            char rotation = (char)(azimuth - 200);
            char stretch = (char)(pitch + 100);
            commandPacket[2] = (byte) (rotation >>> 8);
            commandPacket[3] = (byte) (rotation);
            commandPacket[4] = (byte) (stretch >>> 8);
            commandPacket[5] = (byte) (stretch);

            if (mBtService != null) {
                //mBtService.sendCmd(commandPacket);
            }
        }
        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {

        }
    };


    public static byte exclusiveOr(byte[] commandPacket) {
        Byte result = null;
        return result = (byte) (commandPacket[2] ^ commandPacket[3] ^ commandPacket[4]);

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.

        this.menu = menu;

        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {

            case R.id.menu_connect_bt:

                if (!mBluetoothAdapter.isEnabled()) {
                    Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                    startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
                    return false;
                }

                if (mBtService == null) setupChat();

                if (item.getTitle().equals("ConnectBT")) {
                    // Launch the DeviceListActivity to see devices and do scan
                    serverIntent = new Intent(this, DeviceListActivity.class);
                    startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE);
                } else {
                    if (mBtService != null)
                    {
                        mBtService.stop();
                        item.setTitle(R.string.connectbt);
                    }
                }

                return true;
            case R.id.menu_settings:

                // Launch the DeviceListActivity to see devices and do scan
                serverIntent = new Intent(this, Prefs.class);
                startActivity(serverIntent);

                return true;

            case R.id.menu_clear:

               mConversationArrayAdapter.clear();
                return true;

            case R.id.menu_exit:
                exit();
                return true;

        }

        return super.onOptionsItemSelected(item);
    }

    public void onActivityResult(int requestCode, int resultCode, Intent data) {

        switch (requestCode) {
            case REQUEST_CONNECT_DEVICE:
                // When DeviceListActivity returns with a device to connect
                if (resultCode == MainActivity.RESULT_OK) {
                    connectDevice(data);
                }
                break;

            case REQUEST_ENABLE_BT:

                if (mBtService == null) setupChat();

                if (resultCode == MainActivity.RESULT_OK) {
                    serverIntent = new Intent(this, DeviceListActivity.class);
                    startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE);
                } else {
                    Toast.makeText(getApplicationContext(), "BT device not enabled", Toast.LENGTH_SHORT).show();
                }
                break;
        }
    }

    ///////////////////////////////////////////////////////////////////////

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
    }

    @Override
    public synchronized void onResume() {
        super.onResume();
        getPreferences();
        mSensorManager.registerListener(mSensorEventListener,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION),
                SensorManager.SENSOR_DELAY_NORMAL);
    }

    @Override
    public synchronized void onPause() {
        super.onPause();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();

        if (mBtService != null) mBtService.stop();
        wl.release();
    }

    @Override
    public void onStart() {
        super.onStart();
        getPreferences();
    }

    @Override
    public void onStop() {
        super.onStop();
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        super.onKeyDown(keyCode, event);
        if (keyCode == KeyEvent.KEYCODE_BACK) {

            if (!commandmode) {
                AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(this);
                alertDialogBuilder.setMessage("Are you sure you want exit?");
                alertDialogBuilder.setPositiveButton("Ok",
                        new DialogInterface.OnClickListener() {

                            @Override
                            public void onClick(DialogInterface arg0, int arg1) {
                                exit();
                            }
                        });

                alertDialogBuilder.setNegativeButton("cancel",
                        new DialogInterface.OnClickListener() {

                            @Override
                            public void onClick(DialogInterface arg0, int arg1) {

                            }
                        });

                AlertDialog alertDialog = alertDialogBuilder.create();
                alertDialog.show();
            }
            return false;
        }

        return super.onKeyDown(keyCode, event);
    }

    private void exit() {
        if (mBtService != null) mBtService.stop();
        wl.release();
        android.os.Process.killProcess(android.os.Process.myPid());
    }

    private void getPreferences() {
            SharedPreferences preferences = PreferenceManager
                    .getDefaultSharedPreferences(getBaseContext());
    }

    private void connectDevice(Intent data) {
        tryconnect = true;
        // Get the device MAC address
        String address = data.getExtras().getString(DeviceListActivity.EXTRA_DEVICE_ADDRESS);
        // Get the BluetoothDevice object
        BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
        try {
            // Attempt to connect to the device
            mBtService.connect(device);
            currentdevice = device;

        } catch (Exception e) {
        }
    }

    private void setupChat() {

        // Initialize the BluetoothChatService to perform bluetooth connections
        mBtService = new BluetoothService(this, mBtHandler);

    }

    private void sendMessage(String message) {

       if (mBtService != null)
        {
            // Check that we're actually connected before trying anything
            if (mBtService.getState() != BluetoothService.STATE_CONNECTED) {
                //Toast.makeText(this, R.string.not_connected, Toast.LENGTH_LONG).show();
                return;
            }
            try {
                if (message.length() > 0) {

                    message = message + "\r";
                    // Get the message bytes and tell the BluetoothChatService to write
                    byte[] send = message.getBytes();
                    mBtService.sendCmd(send);
                }
            } catch (Exception e) {
            }
        }
    }

    public static int getKP() {
        return Kp;
    }

    public static void setKP(int KP) {

        mContext1.Kp = KP;
        commandPacketBlueTooth[1] = 0x02;
        commandPacketBlueTooth[2] = 0x01;
        int ikp = KP*100;
        commandPacketBlueTooth[3] = (byte)(ikp / 255);
        commandPacketBlueTooth[4] = (byte)(ikp % 255);;
        //commandPacketBlueTooth[4] = 0x03;
        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
        if (mBtService != null)
            mBtService.sendCmd(commandPacketBlueTooth);
    }

    public static int getKI() {
        return Ki;
    }

    public static void setKI(int KI) {

        mContext1.Ki = KI;
        commandPacketBlueTooth[1] = 0x02;
        commandPacketBlueTooth[2] = 0x02;
        int iki = KI*100;
        commandPacketBlueTooth[3] = (byte)(iki / 255);
        commandPacketBlueTooth[4] = (byte)(iki % 255);;
        //commandPacketBlueTooth[4] = 0x03;
        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
        if (mBtService != null)
            mBtService.sendCmd(commandPacketBlueTooth);
    }

    public static int getKD() {
        return Kd;
    }

    public static void setKD(int KD) {

        mContext1.Kd = KD;
        commandPacketBlueTooth[1] = 0x02;
        commandPacketBlueTooth[2] = 0x03;
        int ikd = KD*20;
        commandPacketBlueTooth[3] = (byte)(ikd / 255);
        commandPacketBlueTooth[4] = (byte)(ikd % 255);;
        //commandPacketBlueTooth[4] = 0x03;
        commandPacketBlueTooth[5] = exclusiveOr(commandPacketBlueTooth);
        if (mBtService != null)
            mBtService.sendCmd(commandPacketBlueTooth);
    }

    public static void setVisible() {
        textView_Speed1.setVisibility(View.VISIBLE);
        textView_Speed2.setVisibility(View.VISIBLE);
        seekBar_Speed1.setVisibility(View.VISIBLE);
        seekBar_Speed2.setVisibility(View.VISIBLE);
    }
}