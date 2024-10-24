#include<string.h>
#include<ESP32Servo.h> // Servo Library 추가

#include <WiFi.h>
#include <PubSubClient.h>

//_____________________ Define And Setting MQTT ______________________
const char* ssid = "Galaxy";
const char* password = "와이파이비밀번호를 넣어주세요";
const char* mqtt_server = "192.168.238.245"; // TOPST 보드의 IP 주소
int mqttPort = 1883;
WiFiClient espClient;
PubSubClient client(espClient);

String TOPIC="JIG";
unsigned long previous_time = 0;
#define interval 1000
char MQTT_data_send[30] = "";

//#include<HardwareSerial.h>

Servo ServoRoTation;    // servo 객체 생성

////______________ Pin Setting (SIGNAL) _______
int MainSTEP_PULSE   = 4;
int MainSTEP_DIR_PIN = 16;
int UD_PulsePin      = 15;
int UD_DirPin        = 2;

////______________ Utility _________________
int ACTUATOR_PUSH_OUTPUT = 33;
int ACTUATOR_PULL_OUTPUT = 32;
int SERVO_ROTATE_PULSE   = 35;
int STEP_UD_PULSE        = 34;

////_____________ ETC ______________________
char Servodir='L';

////______________ TeachPoint _____________
int ServoU_D_Angle_UP   = 0;
int ServoU_D_Angle_DOWN = 0;
int ServoRoTation_ALIGN = 0;
int ServoRoTation_RIGHT = 90;
int ServoRoTation_LEFT  = 0;

////_____________ Tolsin __________________
char SerialData = 'P';

String cmd  = "";
String cmd2 = "";
int SwStatement = -1;

void ConvertSWStatement();
void setup_wifi();
void ServoAMove(Servo sv, int Angle);
void ActuatorWork(int DirInt);// 0 PULL, 1 PUSH
void CheckMovingAsync();
void MoveSTEP(int A); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
void PICTURE_MOVE_RL(char FR);
void PUTMOVE(char DIR);
void GETMOVE(char DIR);
void MQTTcallback(char* topic, byte* payload, unsigned int length);

void setup() {
  
  Serial.begin(115200); // Use Serial Moniter
  Serial.println("INIT");
  delay(50);
  setup_wifi();

  // put your setup code here, to run once:
  pinMode(MainSTEP_PULSE   ,OUTPUT);
  pinMode(MainSTEP_DIR_PIN ,OUTPUT);

  pinMode(ACTUATOR_PUSH_OUTPUT ,OUTPUT);
  pinMode(ACTUATOR_PULL_OUTPUT ,OUTPUT);
  
  pinMode(UD_PulsePin ,OUTPUT);
  pinMode(UD_DirPin   ,OUTPUT);
  
  digitalWrite(ACTUATOR_PUSH_OUTPUT ,LOW);
  digitalWrite(ACTUATOR_PULL_OUTPUT ,LOW);
  
  ServoRoTation.attach(SERVO_ROTATE_PULSE);  
  ServoRoTation.write(0);
  


  cmd  = "";
  cmd2 = "";
  SwStatement = -1000;
}

// if(Serial.available()){
//   cmd2 = Serial.readString();
//   cmd = cmd2;
//   Serial.println(cmd);
//   delay(500);
// }

void ConvertSWStatement(){
    if(cmd != ""){
    if(cmd == "0") SwStatement =0;
    if(cmd == "1") SwStatement =1;
    if(cmd == "3") SwStatement =3;
    if(cmd == "4") SwStatement =4;
    if(cmd == "5") SwStatement =5;
    if(cmd == "6") SwStatement =6;
    if(cmd == "7") SwStatement =7;
    if(cmd == "8") SwStatement =8;
    if(cmd == "9") SwStatement =9;
  }
}

//_________________________________Main LOOP______________________________________________________________________________________________________________________________________________________________________
void loop() {
  delay(30);
  Serial.println(F("INIT"));
  if (!client.connected()) {
    reconnect();
  }else
  {
    delay(100);
    client.loop(); // MQTT New Message Check
    if(cmd != "")
    {
      // put your main code here, to run repeatedly:
      // MQTT 처리영역
      // 기본값
      // JIG1 -> REAR_RIGHT, JIG2 - > REAR_LEFT, JIG3 -> FRONT_RIGHT, JIG4 -> Front_LEFT
      delay(2000);
      ConvertSWStatement();
      delay(500);
      Serial.print("SWStatement: ");
      Serial.println(SwStatement);
      delay(1500);
      
      client.publish("JIG_ESP", "D"); // MoveFinish
      switch(SwStatement){ // 현재 MQTT Message와 동일 함
        case 0: // Init
          Serial.println("Move SW0");
          MoveSTEP(1); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          GETMOVE('L');
          cmd ="";
        break;
        case 1:// GET_REAR_RIGHT
          Serial.println("Move SW1");
          MoveSTEP(2); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          GETMOVE('R');
          MoveSTEP(4); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
        break;
        case 2:// GET_REAR_LEFT
          Serial.println("Move SW2");
          MoveSTEP(2); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          GETMOVE('L');
          MoveSTEP(4); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
        break;
        case 3:// GET_FRONT_RIGHT
          Serial.println("Move SW3");
          MoveSTEP(3); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          GETMOVE('R');
          MoveSTEP(4); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          //ActuatorWork(1);
        break;
        case 4:// GET_FRONT_LEFT
          Serial.println("Move SW4");
          MoveSTEP(3); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          GETMOVE('R');
          MoveSTEP(4); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
        break;
        case 5:// PUT_REAR_RIGHT
          Serial.println("Move SW5");
          MoveSTEP(2); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          PUTMOVE('R');
          MoveSTEP(1); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
        break;
        case 6: // PUT_REAR_LEFT
          Serial.println("Move SW6");
          MoveSTEP(2); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          PUTMOVE('L');
          MoveSTEP(1); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
        break;
        case 7:// PUT_FRONT_RIGHT
          Serial.println("Move SW7");
          MoveSTEP(2); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          PUTMOVE('R');
          MoveSTEP(1); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
        break;
        case 8:// PUT_FRONT_LEFT
          Serial.println("Move SW8");
          MoveSTEP(2); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          PUTMOVE('L');
          MoveSTEP(1); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
        break;
        case 9:// PUT_FRONT_LEFT
          Serial.println("Move SW9");
          delay(500);
          MoveSTEP(2); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          PICTURE_MOVE_RL('F');
          MoveSTEP(3); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
          PICTURE_MOVE_RL('R');
          MoveSTEP(1); // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
        break;
        
        default:
        break;
      }
      // Moving Finish , So Init Data
      cmd ="";
      Serial.println("MoveDone");
      
      
      if (!client.connected()) {
        reconnect();
      }
      delay(1500);
      for(int i =0;i<6;i++){
        client.publish("JIG_ESP", "F"); // MoveFinish
        Serial.print("Send F");
        Serial.println(i);
        delay(1500);
      }
    }
  }
}
//_______________________________________________________________________________________________________________________________________________________________________________________________________

void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  String message;
  Serial.print("Topic : ");
  Serial.println(topic);
  for (int i = 0; i < length; i++) {
    message = message + (char)payload[i];  //Conver *byte to String
  }
  
  Serial.print("Message Is : ");
  Serial.println(message);
  cmd = message;
  //받은 메세지에 따라 팬 상태를 변경한다.
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  // Wi-fi와 연결을 시도한다.
  WiFi.setSleep(false);
  // 연결이 되지 않았을 경우 재시도한다.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.print("Connected to WiFi :");
  Serial.println(WiFi.SSID());
  client.setServer(mqtt_server, mqttPort);
  client.setCallback(MQTTcallback);

  // MQTT 서버와 연결을 시도한다.
  // 연결이 되지 않았을 경우 재시도한다.
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32")) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());  //If you get state 5: mismatch in configuration
      delay(2000);
    }
  }
  //해당 Topic을 구독한다.
  client.subscribe("JIG");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32")) {
      Serial.println("connected");
      client.subscribe("JIG"); // 구독할 토픽
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


void ServoRotation90Angle(char LR){ // 0 ->L , 1-> Align , 2-> R
  int delaytime=0;
  ServoRoTation.attach(SERVO_ROTATE_PULSE);
  if(LR == 'L'){
    ServoRoTation.write(0);
    delay(400);
  }else if(LR == 'R'){
    ServoRoTation.write(180);
    delay(400);
  }
  ServoRoTation.detach();
  delay(1500);
}

int U_D_STEPNowPosition = 0;
int UpPosition = 100;

void GETMOVE(char DIR){ // STEP움직인 이후부터 지그 Get후 Actuator PULL + 서보 정위치 이동 후 Actuator  PULL 까지
    //ServoAMove(ServoU_D,ServoU_D_Angle_DOWN);
    Serial.print("GET Dir Is : ");
    Serial.println(DIR);
    Serial.println("Servo Move");
    if(DIR=='L'){
      ServoRotation90Angle('L');
    }else{
      ServoRotation90Angle('R');
    }
    Serial.println("Actuator PUSH");
    ActuatorWork(1);
    delay(1000);
    // Up Step
    Serial.println("MoveStep Up");
    for(int m = 0; m < UpPosition; m++) {
      digitalWrite(UD_PulsePin,HIGH);
      delay(5);
      digitalWrite(UD_PulsePin,LOW);
      delay(5);
    }
    delay(1000);
    Serial.println("Actuator PULL");
    ActuatorWork(0); // PULL OK

    if(DIR=='L'){
      ServoRotation90Angle('R');
    }else{
      ServoRotation90Angle('L');
    }

    Serial.println("MoveStep Down");
    // UD_DirPin Is OK
    delay(1000);
    digitalWrite(UD_DirPin,HIGH);
    for(int m = 0; m < UpPosition; m++) {
      digitalWrite(UD_PulsePin,HIGH);
      delay(5);
      digitalWrite(UD_PulsePin,LOW);
      delay(5);
    }
    digitalWrite(UD_DirPin,LOW);

    Serial.println("MoveStep Finish");
}

void PUTMOVE(char DIR){ // STEP움직인 이후부터 지그 PUT 후 Actuator PUSH 하고 서보 정위치 이동 후 Actuator  PULL 까지
      //ServoAMove(ServoU_D,ServoU_D_Angle_DOWN);
    Serial.print("PUT Dir Is : ");
    Serial.println(DIR);
    Serial.println("Servo Move");
    if(DIR=='L'){
      ServoRotation90Angle('L');
    }else{
      ServoRotation90Angle('R');
    }
    // Up Step
    Serial.println("MoveStep Up");
    for(int m = 0; m < UpPosition; m++) {
      digitalWrite(UD_PulsePin,HIGH);
      delay(5);
      digitalWrite(UD_PulsePin,LOW);
      delay(5);
    }
    Serial.println("Actuator PUSH");
    ActuatorWork(1);
    delay(1500);

    Serial.println("MoveStep Down");
    digitalWrite(UD_DirPin,HIGH); // Down
    for(int m = 0; m < UpPosition; m++) {
      digitalWrite(UD_PulsePin,HIGH);
      delay(5);
      digitalWrite(UD_PulsePin,LOW);
      delay(5);
    }
    digitalWrite(UD_DirPin,LOW);
    delay(500);

    Serial.println("Actuator PULL");
    ActuatorWork(0); // PULL OK
    delay(1500);

    if(DIR=='L'){
      ServoRotation90Angle('R');
    }else{
      ServoRotation90Angle('L');
    }

    Serial.println("PUT MoveStep Finish");
}

void PICTURE_MOVE_RL(char FR){ // 무조건 R L 순서
  char* L=""; char* R="";
    if(FR == 'F'){L="K"; R="J";} // Message Setting
  else{L="I"; R="H";}
  ServoRotation90Angle('R');
  delay(100);
  client.publish("PICTURE", R); // StartWorking
  delay(3000);
  ServoRotation90Angle('L'); // Align
  delay(100);
  ServoRotation90Angle('L');
  delay(100);
  client.publish("PICTURE", L); // StartWorking
  delay(3000);
  ServoRotation90Angle('R');
  delay(100);
}



//_________________________________________  Utility ___________________________________________________________________

////____________ Position Setting ___________
int InitPosition          = 0;
int JIG_BACKWARD_Position = 0;
int JIG_FORWARD_Position  = 2400;
int PickUp_Position       = 5800;

void MoveSTEP(int A){ // 1-> Init ,2-> JIG_BackWard ,3-> JIG_FORWARD ,4-> PickUpPos
  switch(A){
    case 1: // STEP_MOVE_InitPos_PIO_OUTPUT
      MoveMainConveyor(InitPosition);
      break;
    case 2: // STEP_MOVE_JIG_BACKWARD_PIO_OUTPUT
      MoveMainConveyor(JIG_BACKWARD_Position);
      break;
    case 3: // STEP_MOVE_JIG_FORWARD_PIO_OUTPUT
      MoveMainConveyor(JIG_FORWARD_Position);
      break;
    case 4: // STEP_MOVE_PickUpPos_PIO_OUTPUT
      MoveMainConveyor(PickUp_Position);
      break;
  }
}

int _currentStep = 0;
void MoveMainConveyor(int n){
  // 현재 스텝: 0이라는 가정 -> 모터의 현재 위치 움직여야 하는데
  // n: 목표 위치 , 내가 가야할 위치는 = 내가 원하는 위치 - 내 현재 위치
  // 현재 위치 < 목표 위치 -> 목표 위치까지 앞으로 이동
  // 목표 위치 > 현재 위치 -> 목표 위치까지 뒤로 이동
  // HIGH -> 앞 LOW -> 뒤

       if(_currentStep < n) { digitalWrite(MainSTEP_DIR_PIN, HIGH); delay(1000); } 
  else if(_currentStep > n) { digitalWrite(MainSTEP_DIR_PIN, LOW);  delay(1000); }

  int cur_dist = (n - _currentStep < 0 ? -1 * (n - _currentStep) : n - _currentStep);
  delay(300);
  // 펄스 생성기
  // n만큼 이동
  for(int m = 0; m < cur_dist; m++) {
    digitalWrite(MainSTEP_PULSE,HIGH);
    delay(2);
    digitalWrite(MainSTEP_PULSE,LOW);
    delay(2);
  }

  _currentStep = n;
  delay(50);
  Serial.println("MoveFinish");
  delay(1500);
}

void ActuatorWork(int DirInt)// 0 PULL, 1 PUSH
{
  Serial.print("Actuator : ");
  Serial.println(DirInt);
  switch(DirInt){
    case 0: // PULL
      digitalWrite(ACTUATOR_PUSH_OUTPUT, LOW);
      delay(500);
      digitalWrite(ACTUATOR_PULL_OUTPUT, HIGH);
      //delay(11500);
      delay(5000);
      break;
    case 1: // PUSH
      digitalWrite(ACTUATOR_PUSH_OUTPUT, HIGH);
      delay(500);
      digitalWrite(ACTUATOR_PULL_OUTPUT, LOW);
      //delay(1500);
      delay(5000);
      break;
  }
      digitalWrite(ACTUATOR_PUSH_OUTPUT, LOW);
      delay(50);
      digitalWrite(ACTUATOR_PULL_OUTPUT, LOW);
}
