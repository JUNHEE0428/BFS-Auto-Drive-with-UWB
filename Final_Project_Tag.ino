#include <ArduinoQueue.h>
#include "myFunctions.h"
#include <DW1000Jang.hpp>
#include <DW1000JangUtils.hpp>
#include <DW1000JangTime.hpp>
#include <DW1000JangConstants.hpp>
#include <DW1000JangRanging.hpp>
#include <DW1000JangRTLS.hpp>
#include <Wire.h>
#include <MechaQMC5883.h>


#define Delay 1000

MechaQMC5883 qmc;                           //자이로 센서 라이브러리 클래스 가져오기.


// connection pins
#if defined(ESP8266)
const uint8_t PIN_SS = 15;
#else
const uint8_t PIN_SS = 53; // MEGA spi select pin
//const uint8_t PIN_SS = 10; // UNO spi select pin
const uint8_t PIN_RST = 7;
#endif

//실제 meter 단위로 측정한 거리
double range_A;                             
double range_B;
double range_C;

//실제 meter 단위 기반 좌표
typedef struct Position {
    double x;
    double y;
} Position;

//Anchor 좌표, meter 단위 기반
Position position_A = {0,0};                //Anchor A의 미터 좌표.
Position position_B = {3.2,0};              //Anchor B의 미터 좌표.
Position position_C = {3.2,3.2};            //Anchor C의 미터 좌표. 

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

frame_filtering_configuration_t TAG_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false
};


//삼각 측량 방법으로 위치(meter 기반)를 계산하는 함수
void calculatePosition(double &x, double &y) {

    double A = ( (-2*position_A.x) + (2*position_B.x) );
    double B = ( (-2*position_A.y) + (2*position_B.y) );
    double C = (range_A*range_A) - (range_B*range_B) - (position_A.x*position_A.x) + (position_B.x*position_B.x) - (position_A.y*position_A.y) + (position_B.y*position_B.y);
    double D = ( (-2*position_B.x) + (2*position_C.x) );
    double E = ( (-2*position_B.y) + (2*position_C.y) );
    double F = (range_B*range_B) - (range_C*range_C) - (position_B.x*position_B.x) + (position_C.x*position_C.x) - (position_B.y*position_B.y) + (position_C.y*position_C.y);

    x = (C*E-F*B) / (E*A-B*D);
    y = (C*D-A*F) / (B*D-A*E);
}


//Anchor 통신이 이루어지는 부분, 통신 실패면 false 반환, Anchor와의 통신으로 거리를 얻고 현재 좌표(meter 기반)을 얻는 함수.
Position Get_Current(bool* state) {

  int rand;
  double x,y;
  
  New_structure result_A_Anchor = DW1000JangRTLS::Tag_Distance_Request(1, 1700);
  if(result_A_Anchor.success)
  {

    range_A = result_A_Anchor.distance;
    randomSeed(analogRead(0));
    rand = random(1,10);
    delay(10+10*rand);

    New_structure result_B_Anchor = DW1000JangRTLS::Tag_Distance_Request(2, 1700);
    if(result_B_Anchor.success)
    {

      range_B = result_B_Anchor.distance;
      randomSeed(analogRead(0));
      rand = random(1,10);
      delay(10+10*rand);

      New_structure result_C_Anchor = DW1000JangRTLS::Tag_Distance_Request(3, 1700);
      if(result_C_Anchor.success)
      {

        range_C = result_C_Anchor.distance;
        randomSeed(analogRead(0));
        rand = random(1,10);
        delay(10+10*rand);

        calculatePosition(x,y);
        //Serial.print("Real Coordinates: "); Serial.print(x); Serial.print(","); Serial.println(y);

        Position current = {0, 0};
        current.x = x;
	      current.y = y;
        //Serial.print("Real Coordinates: "); Serial.print(current.x); Serial.print(","); Serial.println(current.y);

        *state = true;                           //A,B,C Anchor와의 통신이 모두 성공한다면 통신 상태의 변수를 true로 바꿈.
        return current;        
      }
    }
  }
  *state = false;                                //A,B,C Anchor와의 통신이 실패한다면 통신 상태 변수를 false로 바꿈.
  return;
}


//3개의 좌표에 대한 이동평균 구하는 함수, 이때 가중치는 똑같이 부여하여 0.33으로함. 윈도우가 움직이는 것은 함수 밖에서 pos1, pos2, pos3의 값을 바꿔주는 형태로 구현함.
Position getWeightedAverage(Position pos1, Position pos2, Position pos3) {
  // Serial.print("Pos1 Coordinates: "); Serial.print(pos1.x); Serial.print(","); Serial.print(pos1.y);
  // Serial.print("Pos2 Coordinates: "); Serial.print(pos2.x); Serial.print(","); Serial.print(pos2.y);
  // Serial.print("Pos3 Coordinates: "); Serial.print(pos3.x); Serial.print(","); Serial.println(pos3.y);
  Position weightedAvg;
  weightedAvg.x = 0.33 * pos1.x + 0.33 * pos2.x + 0.33 * pos3.x;
  weightedAvg.y = 0.33 * pos1.y + 0.33 * pos2.y + 0.33 * pos3.y;
  return weightedAvg;
}


//meter 기반 좌표를 grid 좌표로 변환 해주는 함수.
Point Get_Grid(Position real_cdn) {
  Point grid_cdn = { 0, 0 };
  grid_cdn.col = static_cast<int>(real_cdn.x / Grid_Rate);              //실제 x좌표의 값을 Grid_Rate(하나의 그리드가 차지하는 meter)로 나누어서 그리드 x좌표를 얻는다. 
  grid_cdn.row = static_cast<int>(real_cdn.y / Grid_Rate);              //실제 y좌표의 값을 Grid_Rate로 나누어서 그리드 y좌표를 얻는다. 
  return grid_cdn;
}

//이동평균에서 윈도우를 구현하기 위한 이전 좌표(meter 기반)를 저장하는 변수
Position prePos1 = { 0, 0 };                             //윈도우의 첫번째 칸을 나타내는 변수.
Position prePos2 = { 0, 0 };                             //윈도우의 두번째 칸을 나타내는 변수. 세번째 칸은 얻어온 현재 좌표를 그대로 사용하므로 변수를 따로 선언하지 않음.

Point start = { 1, 0 };                                  //서빙 시작 좌표(그리드 기반)
Point end = { 3, 3 };                                    //첫번째 경로의 목표 좌표(그리드 기반)
ArduinoQueue<Point> end_queue;                           //경로들의 목표 좌표를 저장하는 큐

char outPath[MAX_PATH_LENGTH];                           //하나의 경로에 대한 이동 방향을 저장하는 배열. U,D,R,L
Point outCDN[MAX_PATH_LENGTH];                           //하나의 경로에 대한 이동 좌표를 저장하는 배열. 
char outPathLength;                                      //하나의 경로에 대한 이동 경로 길이를 저장.

//하나의 경로에 대해서 이동 방향을 바탕으로 이동해야 하는 좌표를 저장하는 배열.
void GetPathCDN() {
  Point c = start;
  outCDN[0] = c;
  for (int i = 0; i < outPathLength; i++) {
    for(int j = 0; j < 4; j++)
      if(outPath[i] == directionChars[j]){               //outPath의 이동 방향을 기반으로 방향에 따라 다음 좌표를 얻고 이를 저장.
        c.col += directions[j].col;                      //이동 방향에 따라 col(x)를 수정하여 다음 좌표의 col(x)를 얻음.
        c.row += directions[j].row;                      //이동 방향에 따라 row(y)를 수정하여 다음 좌표의 row(y)를 얻음.
      }
    outCDN[i+1] = c;                                     //outCDN에 다음 좌표를 저장.
  }
}

double goal_deg;                                         //차량이 유지해야 하는 각도.

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  Wire.begin();                                          //자이로 센서와의 연결 시작.
  qmc.init();                                            //자이로 센서 초기화.

  // initialize the driver
  #if defined(ESP8266)
  DW1000Jang::initializeNoInterrupt(PIN_SS);
  #else
  DW1000Jang::initializeNoInterrupt(PIN_SS, PIN_RST);
  #endif
  // general configuration
  DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
  DW1000Jang::enableFrameFiltering(TAG_FRAME_FILTER_CONFIG);

  DW1000Jang::setDeviceAddress(4); // 1조 Device Address : 4
                                    // 2조 Device Address : 5
                                    // ...
                                    // 10조 Device Address : 13
  DW1000Jang::setNetworkId(RTLS_APP_ID);

  DW1000Jang::setAntennaDelay(16436);

  DW1000Jang::setPreambleDetectionTimeout(64);
  DW1000Jang::setSfdDetectionTimeout(273);
  DW1000Jang::setReceiveFrameWaitTimeoutPeriod(8000);

  //모터 제어 핀 출력 설정
  pinMode(LEFT_FRONT_DIR, OUTPUT);
  pinMode(RIGHT_FRONT_DIR, OUTPUT);
  pinMode(RIGHT_BACK_DIR, OUTPUT);
  pinMode(LEFT_BACK_DIR, OUTPUT);
  //PWM 제어 핀 출력 설정
  pinMode(LEFT_FRONT_PWM, OUTPUT);
  pinMode(RIGHT_FRONT_PWM, OUTPUT);
  pinMode(RIGHT_BACK_PWM, OUTPUT);
  pinMode(LEFT_BACK_PWM, OUTPUT);

  ///////////////////////////////////////////////////////
  
  //setup 함수에서 처음 차량 위치 얻어오기.
  int cnt = 0;                                            //처음에는 이동 평균에서의 윈도우에 좌표가 저장되어 있지 않으므로 3개 좌표를 얻어오는 과정에서 횟수를 세주는 변수.
  bool state = true;                                      //Anchor와의 통신의 성공 여부를 저장하는 상태 변수.
  Position current = { 0,0 };                             //현재 좌표(meter 기반)을 저장하는 변수.
  int deg_x, deg_y, deg_z;                                //현재 x,y,z 각도를 저장하는 변수.


  while (cnt < 3) {
    current = Get_Current(&state);                        //Anchor와의 통신으로 현재 좌표(meter 기반)를 얻어옴.
    Serial.print("Real Coordinates: "); Serial.print(current.x); Serial.print(","); Serial.println(current.y);  

    //처음에는 윈도우에 좌표가 저장되어 있지 않으므로 윈도우에 3개의 좌표를 저장시키는 부분.
    if (state) {
      if (cnt == 0) {
        prePos1 = current;                                 //윈도우의 첫번째 칸에 저장.
      }
      else if (cnt == 1) {
        prePos2 = current;                                 //윈도우의 두번째 칸에 저장.
      }
      cnt++;
    }
  }

  current = getWeightedAverage(prePos1, prePos2, current); //이동 평균을 이용해서 현재 좌표를 보정.

  qmc.read(&deg_x, &deg_y, &deg_z);                        //현재 x,y,z 각도 읽기.
  goal_deg = atan2(deg_y, deg_x) * 180 / PI + offset;      //처음 시작하는 차량의 방향을 계산하여 목표 방향으로 설정.

  start = Get_Grid(current);                               //현재 좌표(meter 기반)를 그리드 좌표로 변환하여 시작 그리드 좌표를 얻어옴.
  Serial.print("Start: "); Serial.print((int)start.col); Serial.print(","); Serial.println((int)start.row);
  Point st = start;                                        

  // 각 End Point를 enqueue
  Point points[] = {{5, 4}, {3, 6}, {2, 3}, st};           //모든 경로에 대한 목표 좌표(그리드 기반)를 저장하는 배열.
  for (int i = 0; i < 5; i++) {                           
    end_queue.enqueue(points[i]);                          //목표 좌표를 저장하는 큐에 각 모표 좌표를 저장.
  }

  //시작 좌표부터 목표 좌표까지 최단 경로 구하기.
  end = end_queue.dequeue();                               //첫번째 경로의 목표 좌표를 큐에서 얻기.
  int result = (int)bfs(start, end, outPath, &outPathLength);        //시작 좌표와 목표 좌표, 그리드 맵을 기반으로 최단 경로를 계산.

  if (result == -1) {
    Serial.println("도착점에 도달할 수 없습니다.");         //최단 경로를 계산할 수 없는 경우.
  }
  else {
    GetPathCDN();                                          //BFS를 통해 얻은 "DULR" 방향 정보를 목표 좌표값으로 변환하여 저장.
    Serial.print("경로: ");
    for (int i = 0; i < outPathLength; i++) {
      Serial.print(outPath[i]);                            
    }
    Serial.println();

    Serial.print("Grid 좌표: ");
    for (int i = 0; i <= outPathLength; i++) {
      Serial.print("("); Serial.print((int)outCDN[i].col); Serial.print(","); Serial.print((int)outCDN[i].row); Serial.print(") ");
    }
    Serial.println();
  }
}


char index = 0;                                               //하나의 경로에는 여러 경유지가 존재하고 현재 어떤 경유지를 이동하는지를 나타내는 인덱스.
bool finish_flag = false;                                     //하나의 경로의 목표 좌표에 도달하였는지를 저장하는 상태 변수.



void loop() {
  bool state = true;                                          // Anchor와 통신 성공 여부를 저장하는 상태 변수
  Point curGrid = { 0,0 };                                    // 현재 격자 좌표를 저장할 변수 선언
  Position curAvg = { 0,0 };                                  // 현재 좌표의 이동 평균 값을 저장할 변수 선언
  Position current = Get_Current(&state);                     // 통신을 통해 현재 좌표값을 얻고 통신이 실패했을 경우 state에 false 저장
  
  int deg_x, deg_y, deg_z;                                    // 센서에서 읽어온 각도를 저장할 변수 선언

  qmc.read(&deg_x, &deg_y, &deg_z);                           // 센서에서 각도를 읽어와 저장
  heading_deg = atan2(deg_y, deg_x) * 180 / PI;               // 읽어온 각도값을 바탕으로 현재 각도 계산

  if (state) {
    curAvg = getWeightedAverage(prePos1, prePos2, current);   // 이동 평균을 구하여 저장
    //Serial.print("Real Coordinates: "); Serial.print(curAvg.x); Serial.print(","); Serial.println(curAvg.y);

    prePos1 = prePos2;                                        
    prePos2 = current;                                        // 이동 평균을 구하기 위해 이전 위치값을 저장

    curGrid = Get_Grid(curAvg);                               // 이동 평균값을 격자 좌표로 변환

    Serial.print("Grid Coordinate: "); Serial.print("("); Serial.print((int)curGrid.col); Serial.print(","); Serial.print((int)curGrid.row); Serial.println(")");
    //Serial.print("Grid Coordinates1: "); Serial.print((int)(curAvg.x * 10 + 0.3)); Serial.print(","); Serial.println((int)(curAvg.y * 10 + 0.3));
    //Serial.print("Grid Coordinates2: "); Serial.print((int)(outCDN[index].col * Grid_Rate * 10)); Serial.print(","); Serial.println((int)(outCDN[index].row * Grid_Rate * 10));

    if (((curAvg.x >= (outCDN[index].col + 0.5) * Grid_Rate - Grid_Rate * 0.25) && (curAvg.x <= (outCDN[index].col + 0.5) * Grid_Rate + Grid_Rate * 0.25)) 
      && ((curAvg.y >= (outCDN[index].row + 0.5) * Grid_Rate - Grid_Rate * 0.25) && (curAvg.y <= (outCDN[index].row + 0.5) * Grid_Rate + Grid_Rate * 0.25))) {
      //목표 그리드에 도착했을 때 
      Stop();
      if (finish_flag) {
        //모든 목표를 끝냈을 때
        Stop();
      }
      else if (index == outPathLength) {
        //End에 도달했을 때
        if (!end_queue.isEmpty()) {
          start = end;
          end = end_queue.dequeue();                                    // 목적지 배열에서 다음 목적지를 꺼내 저장
          int result = (int)bfs(start, end, outPath, &outPathLength);   // 해당 목적지 도달 가능 여부 및 경로 정보를 저장
          
          while (result == -1) {                                        // 도달 가능한 목적지가 나오지 않으면 반복
            end = end_queue.dequeue();                                  
            result = (int)bfs(start, end, outPath, &outPathLength);     
          }

          GetPathCDN();         // BFS를 통해 얻은 "DULR" 방향 정보를 목표 좌표값으로 변환하여 저장

          index = 0;            // 다음 목적지까지 이동하기 위하여 index 초기화

          delay(2000);          // 다음 목적지로 출발하기 전 2초 정지
        }
        else {
          finish_flag = true;   // 모든 경로를 순회하였음을 알려주는 flag
        }
      }
      else {
        if(heading_deg > goal_deg + 5){       // 각도가 기준보다 오른쪽으로 5도 이상 틀어졌으면
          TurnLeft();                         // 왼쪽으로 회전
        }
        else if(heading_deg < goal_deg - 5){  // 각도가 기준보다 왼쪽으로 5도 이상 틀어졌으면
          TurnRight();                        // 오른쪽으로 회전
        }
        else{                     // 각도가 틀어지지 않았으면
        delay(1000);              // 1초 정지 후 다음 동작
        switch(outPath[index]) {  // 다음 경로의 방향에 따라 다르게 동작
          case 'D':               // Down (map 기준이므로 실제로는 Up 동작)
            MoveUp();
            break;
          case 'U':               // Up (map 기준이므로 실제로는 Down 동작)
            MoveDown();
            break;
          case 'L':               // Left 방향 동작
            MoveLeft();
            break;
          case 'R':               // Right 방향 동작
            MoveRight();
            break;
          default:
            break;
          }
          index++;                // 다음 경로로 이동
        }
      }
    }
  }
}

