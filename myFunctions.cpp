#include "myFunctions.h"
#include <Arduino.h>
#include <ArduinoQueue.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//그리드 8*8 3.2m*3.2m GridRate 3.2/8m
char grid[ROWS][COLS] = {
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 0, 0, 0, 0, 0, 1}, 
  {1, 1, 1, 1, 0, 1, 0, 1},
  {1, 1, 0, 0, 0, 1, 0, 1}, 
  {1, 1, 0, 1, 0, 0, 0, 1}, 
  {1, 1, 0, 0, 0, 1, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 1}
};

struct Node {
  Point pt; // 현재 좌표
  char path[MAX_PATH_LENGTH]; // 경로 배열
  char pathLength; // 경로 길이
};

Point directions[DIRSIZE] = {
  { -1, 0 }, // 위
  { 1, 0 }, // 아래
  { 0, -1 }, // 왼쪽
  { 0, 1 } // 오른쪽
};

char directionChars[DIRSIZE] = {'U', 'D', 'L', 'R'};

double Grid_Rate = 3.2/8;

char bfs(Point start, Point end, char* outPath, char* outPathLength) {  // BFS 알고리즘을 이용하여 최단 경로를 계산하고 저장하는 함수
  if (grid[start.row][start.col] == 1 || grid[end.row][end.col] == 1) { // 시작점 또는 도착점이 장애물일 경우 도착할 수 없음
    return -1;
  }

  ArduinoQueue<Node> queue;                                             // BFS 알고리즘을 사용하기 위한 queue 선언
  Node startNode = {start, "", 0};                                      // 
  queue.enqueue(startNode);                                             // 시작 노드 생성 후 queue에 삽입

  int distance[ROWS][COLS];                                             // 각 좌표까지의 거리를 저장하기 위하여 map과 같은 크기의 배열 생성
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
      distance[i][j] = -1;                                              // 경로에 도착하지 않았음을 표현하기 위해 거리값을 -1로 초기화
    }
  }
  distance[start.row][start.col] = 0;                                   // 시작점의 거리 0으로 저장

  while (!queue.isEmpty()) {                                            // queue가 비어있을 때(BFS 끝)까지 반복
    Node current = queue.dequeue();                                     // 현재 위치를 dequeue 값으로 저장

    if (current.pt.row == end.row && current.pt.col == end.col) {       // 목표 위치에 도달하면 현재까지 저장한 경로 반환
      //Serial.print("최단 경로 길이: ");
      //Serial.println(distance[current.pt.row][current.pt.col]);
      memcpy(outPath, current.path, current.pathLength);                // 현재까지 경로 정보를 외부 변수에 저장
      *outPathLength = current.pathLength;                              // 거리값 저장
      return distance[current.pt.row][current.pt.col];                  
    }

    for (int i = 0; i < 4; i++) {
      Point next = {current.pt.row + directions[i].row, current.pt.col + directions[i].col};    // 이동 가능한 방향(4 방향)을 모두 고려

      if (next.row >= 0 && next.row < ROWS && next.col >= 0 && next.col < COLS &&
          grid[next.row][next.col] == 0 && distance[next.row][next.col] == -1) {                // 범위를 벗어나거나 장애물이 있으면 고려하지 않음
        Node nextNode = {next, "", current.pathLength + 1};                                     // 좌표와 거리값을 저장한 새로운 노드 생성
        memcpy(nextNode.path, current.path, current.pathLength);                                //
        nextNode.path[current.pathLength] = directionChars[i];                                  // 위 노드에 현재까지 경로 저장
        queue.enqueue(nextNode);                                                                // 노드 enqueue
        distance[next.row][next.col] = distance[current.pt.row][current.pt.col] + 1;            // 해당 좌표의 거리값 갱신
      }
    }
  }

  return -1; // 도착점에 도달할 수 없는 경우
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//모터 파라미터 설정
const char LEFT_FRONT_PWM = 5; // motor 1 speed (0~255) -- LEFT FRONT WHEEL
const char LEFT_FRONT_DIR = 4; // motor 1 direction (0 or 1)

const char RIGHT_FRONT_PWM = 6; // motor 2 speed (0~255) -- RIGHT FRONT WHEEL
const char RIGHT_FRONT_DIR = A1; // motor 2 direction (0 or 1)

const char RIGHT_BACK_PWM = 9; // motor 3 speed (0~255) -- RIGHT BACK WHEEL
const char RIGHT_BACK_DIR = 8; // motor 3 direction (0 or 1)

const char LEFT_BACK_PWM = 3; // motor 4 speed (0~255) -- LEFT BACK WHEEL
const char LEFT_BACK_DIR = A2; // motor 4 direction (0 or 1)

//처음 위치 heading 방향은 +y축 방향으로 기준
//위로 움직이는 함수
void MoveUp() {
  analogWrite(LEFT_FRONT_PWM, Speed);  
  digitalWrite(LEFT_FRONT_DIR, HIGH);

  analogWrite(RIGHT_FRONT_PWM, Speed);  
  digitalWrite(RIGHT_FRONT_DIR, HIGH);

  analogWrite(RIGHT_BACK_PWM, Speed);  
  digitalWrite(RIGHT_BACK_DIR, HIGH);

  analogWrite(LEFT_BACK_PWM, Speed);  
  digitalWrite(LEFT_BACK_DIR, HIGH);
}
//아래로 움직이는 함수
void MoveDown() {
  analogWrite(LEFT_FRONT_PWM, Speed);  
  digitalWrite(LEFT_FRONT_DIR, LOW);

  analogWrite(RIGHT_FRONT_PWM, Speed);  
  digitalWrite(RIGHT_FRONT_DIR, LOW);

  analogWrite(RIGHT_BACK_PWM, Speed);  
  digitalWrite(RIGHT_BACK_DIR, LOW);

  analogWrite(LEFT_BACK_PWM, Speed);  
  digitalWrite(LEFT_BACK_DIR, LOW);
}
//오른쪽으로 움직이는 함수
void MoveRight() {
  analogWrite(LEFT_FRONT_PWM, Speed);  
  digitalWrite(LEFT_FRONT_DIR, HIGH);

  analogWrite(RIGHT_FRONT_PWM, Speed);  
  digitalWrite(RIGHT_FRONT_DIR, LOW);

  analogWrite(RIGHT_BACK_PWM, Speed);  
  digitalWrite(RIGHT_BACK_DIR, HIGH);

  analogWrite(LEFT_BACK_PWM, Speed);  
  digitalWrite(LEFT_BACK_DIR, LOW);
}
//왼쪽으로 이동하는 함수
void MoveLeft() {
  analogWrite(LEFT_FRONT_PWM, Speed);  
  digitalWrite(LEFT_FRONT_DIR, LOW);

  analogWrite(RIGHT_FRONT_PWM, Speed);  
  digitalWrite(RIGHT_FRONT_DIR, HIGH);

  analogWrite(RIGHT_BACK_PWM, Speed);  
  digitalWrite(RIGHT_BACK_DIR, LOW);

  analogWrite(LEFT_BACK_PWM, Speed);  
  digitalWrite(LEFT_BACK_DIR, HIGH);
}
//오른쪽으로 회전하는 함수
void TurnRight() {
  analogWrite(LEFT_FRONT_PWM, Speed);  
  digitalWrite(LEFT_FRONT_DIR, HIGH);

  analogWrite(RIGHT_FRONT_PWM, Speed);  
  digitalWrite(RIGHT_FRONT_DIR, LOW);

  analogWrite(RIGHT_BACK_PWM, Speed);  
  digitalWrite(RIGHT_BACK_DIR, LOW);

  analogWrite(LEFT_BACK_PWM, Speed);  
  digitalWrite(LEFT_BACK_DIR, HIGH);
}
//왼쪽으로 회전하는 함수
void TurnLeft() {
  analogWrite(LEFT_FRONT_PWM, Speed);  
  digitalWrite(LEFT_FRONT_DIR, LOW);

  analogWrite(RIGHT_FRONT_PWM, Speed);  
  digitalWrite(RIGHT_FRONT_DIR, HIGH);

  analogWrite(RIGHT_BACK_PWM, Speed);  
  digitalWrite(RIGHT_BACK_DIR, HIGH);

  analogWrite(LEFT_BACK_PWM, Speed);  
  digitalWrite(LEFT_BACK_DIR, LOW);
}
//정지하는 함수
void Stop() {
  analogWrite(RIGHT_FRONT_PWM, 0);  
  analogWrite(RIGHT_BACK_PWM, 0);  
  analogWrite(LEFT_BACK_PWM, 0);  
  analogWrite(LEFT_FRONT_PWM, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////