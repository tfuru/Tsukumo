#include <SoftwareSerial.h>
#include <TinyWireM.h>
#include <USI_TWI_Master.h>
#include <TinyWireM_Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// this is the 'minimum' pulse length count (out of 4096)
#define SERVOMIN  150
// this is the 'maximum' pulse length count (out of 4096)
#define SERVOMAX  600 

//rx,tx
SoftwareSerial serial(3,4);

const int CMD_MAX_SIZE = 3;
unsigned char cmdData[CMD_MAX_SIZE];
int index = 0;

void setup() {
  //SoftwareSerial 初期化
  serial.begin(115200);

  //サーボドライバ初期化
  pwm.begin();
  pwm.setPWMFreq(60);
  yield();
}

//角度をパルス幅に変換
// 0 - 180
uint16_t and2pulselen(unsigned char ang){
  return map(ang, 0, 180, SERVOMIN, SERVOMAX);
}

//コマンドを解析してサーボ,角度を設定して動かす
/** コマンドフォーマット
 * cmdData[0] : コマンドタイプ 0:サーボ
 * cmdData[1] : サーボ番号
 * cmdData[2] : 角度
 */
void onCmd(unsigned char cmdData[]){
  //サーボ番号
  unsigned char type = cmdData[0];
  if(type == 0){
    //サーボ番号
    uint8_t num = cmdData[1];
    //角度
    unsigned char ang = cmdData[2];
    
    //角度をパルス幅に変換
    uint16_t pulselen = and2pulselen(ang);
    
    //サーボの角度変更
    pwm.setPWM(num, 0, pulselen);
  }
}

//テスト
void test(){
  cmdData[0] = 0;
  cmdData[1] = 0;
  cmdData[2] = 180;
  onCmd(cmdData);
  delay(3000);
  
  cmdData[0] = 0;
  cmdData[1] = 0;
  cmdData[2] = 0;
  onCmd(cmdData);
  delay(3000);  
}

void loop() {
  //test();
  //Serialから値が届くのを待つ
  if(serial.available()){
    cmdData[index] = serial.read();
    index++;
    if(index >= CMD_MAX_SIZE){
      //全てのデータを読みだしたのでコマンドとして実行
      onCmd(cmdData);
      index = 0;
    }
  }
}
