#include <ros/ros.h>
#include <linux/input.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

std::map<char, std::vector<float>> moveBindings
{
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
}; // 방향키 바인딩

std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
}; // 속도키 바인딩

const char* msg = R"(
다음 키로 조종하세요!
---------------------------
   u(↖) | i(↑) | o(↗)
   ---------------------
   j(←) |k(정지)| l(→)
   ---------------------
   m(↙) | ,(↓) | .(↘)
---------------------------
t : 위 (+z)
b : 아래 (-z)

q/z : 10%씩 최대속도 증감
w/x : 10%씩 선형속도 증감
e/c : 10%씩 회전속도 증감

종료하려면 CTRL+C를 누르세요.
)"; // 안내 메시지(선형속도 : 직선을 따라 이동 시 물체의 위치 변화율)

float speed(0.5); // 선형속도 (m/s)
float turn(1.0); // 회전속도 (rad/s)
float x(0), y(0), z(0), th(0);
char key(' ');

int getch(void){
  int ch;
  struct termios oldt;
  struct termios newt;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt; // 이전 설정 백업 및 새 설정 덮어쓰기

  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt); // 변경 사항 새 설정에 적용

  ch = getchar(); // 입력한 문자 얻기
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 이전 설정 재적용

  return ch;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "operate");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  geometry_msgs::Twist twist;

  printf("%s", msg);
  printf("\r현재 속도와 회전 : %f\t%f | 명령 대기 중...\r", speed, turn);

  while(true){
    key = getch(); // 키 입력

    if (moveBindings.count(key) == 1){
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
      th = moveBindings[key][3];
      printf("\r현재 속도와 회전 : %f\t%f | 입력한 명령 : %c", speed, turn, key);
    } // 방향 키 입력 시
    else if (speedBindings.count(key) == 1){
      speed = speed * speedBindings[key][0];
      turn = turn * speedBindings[key][1];
      printf("\r현재 속도와 회전 : %f\t%f | 입력한 명령 : %c", speed, turn, key);
    } // 속도 키 입력 시
    else{
      x = 0;
      y = 0;
      z = 0;
      th = 0;

      if (key == '\x03'){ // ctrl-C 입력 시 종료.
        printf("\n\n안뇽~.\n\n");
        break;
      }

      printf("\r현재 속도와 회전 : %f\t%f | 무효한 명령! %c", speed, turn, key);
    } // 이외의 키 입력시 멈춤

    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn; // 메시지 업데이트

    pub.publish(twist); // 메시지 퍼블리싱
    ros::spinOnce();
  }

  return 0;
}
