package com.leslieliu.drivingalarm;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.media.AudioManager;
import android.media.MediaPlayer;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import java.io.IOException;
import java.io.InputStream;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    private final static String MY_UUID = "00001101-0000-1000-8000-00805F9B34FB";   //SPP服务UUID号
    private BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter(); //获取蓝牙实例
    private String smsg = ""; //显示用数据缓存
    private TextView textView;
    private MediaPlayer mediaPlayer;
    private int currentAction = 0;

    BluetoothDevice mBluetoothDevice = null; //蓝牙设备
    BluetoothSocket mBluetoothSocket = null; //蓝牙通信Socket

    boolean bRun = true; //运行状态
    boolean bThread = false; //读取线程状态
    private InputStream is;    //输入流，用来接收蓝牙数据

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        textView = findViewById(R.id.textView);

        //如果打不开蓝牙提示信息，结束程序
        if (mBluetoothAdapter == null) {
            Toast.makeText(getApplicationContext(), "无法打开手机蓝牙，请确认手机是否有蓝牙功能！", Toast.LENGTH_SHORT).show();
            finish();
            return;
        }

        //连接按钮响应
        final Button connectButton = findViewById(R.id.connectButton);
        connectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (mBluetoothAdapter.isEnabled() == false) {
                    Toast.makeText(getApplicationContext(), " 请先打开蓝牙", Toast.LENGTH_LONG).show();
                    return;
                }

                //如果未连接设备则打开DevicesListActivity搜索设备
                if (mBluetoothSocket == null) {
                    Intent serveIntent = new Intent(getApplicationContext(), DevicesListActivity.class); //跳转活动
                    startActivityForResult(serveIntent, 1); //设置返回宏定义
                } else {
                    //关闭连接socket
                    try {
                        bRun = false;
                        Thread.sleep(2000);

                        is.close();
                        mBluetoothSocket.close();
                        mBluetoothSocket = null;

                        connectButton.setText("连接帽子");
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }

                }
            }
        });


        // 设置设备可以被搜索
        new Thread() {
            @SuppressLint("MissingPermission")
            public void run() {
                if (!mBluetoothAdapter.isEnabled()) {
                    mBluetoothAdapter.enable();
                }
            }
        }.start();
    }

    //接收活动结果，响应startActivityForResult()
    @SuppressLint("MissingPermission")
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        switch (requestCode) {
            case 1:     //连接结果，由DeviceListActivity设置返回
                // 响应返回结果
                if (resultCode == Activity.RESULT_OK) {   //连接成功，由DeviceListActivity设置返回
                    // MAC地址，由DeviceListActivity设置返回
                    String address = data.getExtras().getString(DevicesListActivity.EXTRA_DEVICE_ADDRESS);
                    // 得到蓝牙设备句柄
                    mBluetoothDevice = mBluetoothAdapter.getRemoteDevice(address);

                    // 用服务号得到socket
                    try {
                        mBluetoothSocket = mBluetoothDevice.createRfcommSocketToServiceRecord(UUID.fromString(MY_UUID));
                    } catch (IOException e) {
                        Toast.makeText(this, "连接失败！", Toast.LENGTH_SHORT).show();
                    }
                    //连接socket
                    Button connectButton = findViewById(R.id.connectButton);
                    try {
                        mBluetoothSocket.connect();
                        Toast.makeText(this, "连接" + mBluetoothDevice.getName() + "成功！", Toast.LENGTH_SHORT).show();
                        connectButton.setText("退出登录");
                    } catch (IOException e) {
                        try {
                            Toast.makeText(this, "连接失败！", Toast.LENGTH_SHORT).show();
                            mBluetoothSocket.close();
                            mBluetoothSocket = null;
                        } catch (IOException ee) {
                            Toast.makeText(this, "连接失败！", Toast.LENGTH_SHORT).show();
                        }

                        return;
                    }

                    //打开接收线程
                    try {
                        is = mBluetoothSocket.getInputStream();   //得到蓝牙数据输入流
                    } catch (IOException e) {
                        Toast.makeText(this, "接收数据失败！", Toast.LENGTH_SHORT).show();
                        return;
                    }
                    if (bThread == false) {
                        readThread.start();
                        bThread = true;
                    } else {
                        bRun = true;
                    }
                }
                break;
            default:
                break;
        }
    }

    //接收数据线程
    Thread readThread = new Thread() {

        public void run() {
            byte[] buffer = new byte[1024];
            bRun = true;
            //接收线程
            while (true) {
                try {
                    while (is.available() == 0) {
                        while (bRun == false) {
                        }
                    }
                    while (true) {
                        if (!bThread)//跳出循环
                            return;

                        is.read(buffer);         //读入数据
                        smsg = new String(buffer);   //写入接收缓存
                        if (is.available() == 0) break;  //短时间没有数据才跳出进行显示
                    }
                    //发送显示消息，进行显示刷新
                    handler.sendMessage(handler.obtainMessage());
                } catch (IOException e) {
                }
            }
        }
    };

    //消息处理队列
    @SuppressLint("HandlerLeak")
    Handler handler = new Handler() {
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            takeAction(smsg.charAt(0) - '0');
        }
    };

    //关闭程序掉用处理部分
    public void onDestroy() {
        super.onDestroy();
        if (mBluetoothSocket != null)  //关闭连接socket
            try {
                mBluetoothSocket.close();
            } catch (IOException e) {
            }
        //	_bluetooth.disable();  //关闭蓝牙服务
    }

    private void takeAction(int sign) {
        if (sign != currentAction) {
            if (mediaPlayer != null) {
                mediaPlayer.release();
                mediaPlayer = null;
            }
            if (sign == 0) textView.setText("无消息");
            else if (sign == 1) {
                mediaPlayer = MediaPlayer.create(this, R.raw.alarm);
                mediaPlayer.setLooping(true);
                mediaPlayer.start();
                textView.setText("小心疲劳驾驶！");
            } else if (sign == 2) {
                mediaPlayer = MediaPlayer.create(this, R.raw.music);
                mediaPlayer.setLooping(true);
                mediaPlayer.start();
                textView.setText("累了？来点音乐！");
            } else if (sign == 3) {
                mediaPlayer = MediaPlayer.create(this, R.raw.ringtone);
                mediaPlayer.setLooping(true);
                mediaPlayer.start();
                textView.setText("危险！小心睡着！");
            } else if (sign == 4) {
                mediaPlayer = MediaPlayer.create(this, R.raw.alarm);
                mediaPlayer.setLooping(true);
                mediaPlayer.start();
                textView.setText("行驶途中请勿侧身！");
            } else if (sign == 5) { //未使用
                textView.setText("无消息");
            }
            currentAction = sign;
        }
    }
}
