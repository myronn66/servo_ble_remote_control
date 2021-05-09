#include <SoftwareSerial.h>
#include <Servo.h>

Servo myservo;

// 定義連接藍牙模組的序列埠
SoftwareSerial BT(8, 9); // RX, TX // uno3 pin 8, pin 9
char ch_ser, ch_bt;  // 儲存接收資料的變數

// 指令格式為 'bt0' ~ 'bt9'
int chk_cmd = 0;
int cmd_idx = 0;
int uno_cmd = 99; // null data

// LED 10,11,12
int LEDRpin = 10; // pin 10
int LEDGpin = 11; // pin 11
int LEDBpin = 12; // pin 12

void set_led_onoff(int r, int g, int b)
{
    digitalWrite(LEDRpin, r);
    digitalWrite(LEDGpin, g);
    digitalWrite(LEDBpin, b);  
}

void setup()
{
    // Init LED
    pinMode(LEDRpin, OUTPUT);
    pinMode(LEDGpin, OUTPUT);
    pinMode(LEDBpin, OUTPUT);
    set_led_onoff(LOW, LOW, LOW);

    // Init BLE & Serial
    Serial.begin(9600);   // 與電腦序列埠連線
    Serial.println("BT is ready!");
 
    // 設定藍牙模組的連線速率
    BT.begin(115200);

    // Init Servo
    myservo.attach(2);  // 設定要將伺服馬達接到PIN2
    myservo.attach(2, 500, 2447); // 修正脈衝寬度範圍
    myservo.write(90); // 一開始先置中90度 // STOP
}

void loop()
{
    // 若收到「序列埠監控視窗」的資料，則送到藍牙模組 // PC Serial TX ---> BLE RX
    if (Serial.available())
    {
        ch_ser = Serial.read();
        BT.println(ch_ser);
    }

    // 若收到藍牙模組的資料，則送到「序列埠監控視窗」 // BLE TX ---> PC Serial RX
    if (BT.available())
    {
        ch_bt = BT.read();
        //Serial.println(ch_bt);

        // check 'b'
        if (chk_cmd == 0)
        {
            if (ch_bt == 'b')
            {
                chk_cmd = 1;
                cmd_idx = 1;
                return;
            }
            else
            {
                chk_cmd = 0;
                cmd_idx = 0;
            }
        }

        if (chk_cmd == 1)
        {
            if (cmd_idx == 1)
            {
                // check 't'
                if (ch_bt == 't')
                {
                    cmd_idx = 2;
                    return;
                }
                else if (ch_bt != 0x00)
                {
                    chk_cmd = 0;
                    cmd_idx = 0;
                }
            }
            else if (cmd_idx == 2)
            {
                // check command id
                switch (ch_bt)
                {
                    case '0': /* Serial.println("Command: 0"); */ uno_cmd = 0; break;
                    case '1': /* Serial.println("Command: 1"); */ uno_cmd = 1; break;
                    case '2': /* Serial.println("Command: 2"); */ uno_cmd = 2; break;
                    case '3': /* Serial.println("Command: 3"); */ uno_cmd = 3; break;
                    case '4': /* Serial.println("Command: 4"); */ uno_cmd = 4; break;
                    case '5': /* Serial.println("Command: 5"); */ uno_cmd = 5; break;
                    case '6': /* Serial.println("Command: 6"); */ uno_cmd = 6; break;
                    case '7': /* Serial.println("Command: 7"); */ uno_cmd = 7; break;
                    case '8': /* Serial.println("Command: 8"); */ uno_cmd = 8; break;
                    case '9': /* Serial.println("Command: 9"); */ uno_cmd = 9; break;
                    default:
                        uno_cmd = 99;
                        break;
                }
                chk_cmd = 0;
                cmd_idx = 0;
            }

            // Serial.println(uno_cmd); // print command id

            switch (uno_cmd)
            {
                case 0: // STOP // Init Servo motor
                    myservo.write(90);  // sg90 [STOP]
                    //delay(1000);
                    uno_cmd = 99;
                    break;

                case 1:  // control servo motor
                    //myservo.write(97); // 逆時針
                    //delay(220); // 60'// delay(110); 45' //delay(220); 90' // 2.4ms=1'
                    //myservo.write(90);
                    //delay(2000);
                    myservo.write(85); // 順時針
                    delay(600); // 350ms=90' // 3.89ms=1'
                    myservo.write(90);
                    uno_cmd = 99;
                    break;

                case 2:  // R/G/B LED On/Off
                    set_led_onoff(HIGH, LOW,  LOW);  delay(500);
                    set_led_onoff(LOW,  HIGH, LOW);  delay(500);
                    set_led_onoff(LOW,  LOW,  HIGH); delay(500);
                    set_led_onoff(HIGH, HIGH, LOW);  delay(500);
                    set_led_onoff(LOW,  HIGH, HIGH); delay(500);
                    set_led_onoff(HIGH, HIGH, HIGH); delay(500);
                    uno_cmd = 99;
                    break;

                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                default:
                    uno_cmd = 99;
                    return;
            }
        }
    }
}
