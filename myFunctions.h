#ifndef MYFUNCTIONS_H
#define MYFUNCTIONS_H

#define ROWS 8                                              //그리드 맵의 행(y).
#define COLS 8                                              //그리드 맵의 열(x).
#define MAX_PATH_LENGTH 100                                 //최대 최단 경로의 길이.
#define DIRSIZE 4                                           //위, 아래, 오른쪽, 왼쪽 방향 배열의 크기.

#define Speed 70                                            //모터 PWM 값.

//모터 핀 번호.
//'D'
#define TRIG1 23
#define ECHO1 22

//'U'
#define TRIG2 25
#define ECHO2 24

//'R'
#define TRIG3 29
#define ECHO3 28

//'L'
#define TRIG4 27
#define ECHO4 26

//그리드 좌표 구조체.
typedef struct Point {
  char row;
  char col;
} Point;

extern char grid[ROWS][COLS];                                            //그리드 맵 변수.
extern Point directions[DIRSIZE];                                        //위, 아래, 오른쪽, 왼쪽을 나타내는 그리드 좌표 배열.
extern char directionChars[DIRSIZE];                                     //위, 아래, 오른쪽, 왼쪽을 나타내는 문자 배열.
extern double Grid_Rate;                                                 //하나의 그리드가 차지하는 meter.

extern const char LEFT_FRONT_PWM;
extern const char LEFT_FRONT_DIR;

extern const char RIGHT_FRONT_PWM;
extern const char RIGHT_FRONT_DIR;

extern const char RIGHT_BACK_PWM;
extern const char RIGHT_BACK_DIR;

extern const char LEFT_BACK_PWM;
extern const char LEFT_BACK_DIR;

char bfs(Point start, Point end, char* outPath, char* outPathLength);    //최단 경로를 계산하는 BFS 알고리즘 함수.

//모터 제어 함수.
void MoveUp();
void MoveDown();
void MoveLeft();
void MoveRight();
void TurnLeft();
void TurnRight();
void Stop();

#endif