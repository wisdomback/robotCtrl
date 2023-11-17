#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

float speed(0.5); // 선형속도 (m/s)
float turn(1.0); // 회전속도 (rad/s)
float x(0), y(0), z(0), th(0);

struct axis_state {
    short x, y;
}; // 아날로그의 현 상태

int readEvent(int fd, struct js_event *event){
    ssize_t bytes;
    bytes = read(fd, event, sizeof(*event));
    if (bytes == sizeof(*event)) return 0; // 이벤트 성공 시.
    return -1; // 이벤트 못 읽음
} // 컨트롤러에서 이벤트 읽기

size_t getAxis(struct js_event *event, struct axis_state axes[3]){
    size_t axis = event->number / 2;
    if (axis < 3){
        if (event->number % 2 == 0) axes[axis].x = event->value;
        else axes[axis].y = event->value;
    }
    return axis;
} // 아날로그 x,y값을 리턴

size_t getBtnCnt(int fd){
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1) return 0;
    return buttons;
}

void action(struct js_event event){
  if(event.value == 1){
    if(event.number == 0){
      printf("앞으로 주행(A)\n");
      x = 1;
    }
    else if(event.number == 1){
      printf("뒤로 주행(B)\n");
      x = -1;
    }
    else if(event.number == 4){
      printf("왼쪽으로 회전(LB)\n");
      y = 1;
      th = 1;
    }
    else if(event.number == 5){
      printf("오른쪽으로 회전(RB)\n");
      y = -1;
      th = -1;
    }
    else if(event.number == 8){
      printf("수동 종료(Home)\n");
      exit(0);
    }
    else if(event.number == 2){
      printf("속도 감소(X)\n");
      speed -= 1.0;
    }
    else if(event.number == 3){
      printf("속도 증가(Y)\n");
      speed += 1.0;
    }
    else if(event.number == 6){
      printf("회전 감소(Select)\n");
      turn -= 1.0;
    }
    else if(event.number == 7){
      printf("회전 증가(Start)\n");
      turn += 1.0;
    }
  }

  if(event.value == 0){
    if(event.number == 0 || event.number == 1 || event.number == 4 || event.number == 5){
      printf("멈춤\n");
      x = 0;
      y = 0;
      z = 0;
      th = 0;
    }
  }
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "operate");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    geometry_msgs::Twist twist;

    const char *device;
    int js;
    struct js_event event; // 버튼 이벤트
    struct axis_state axes[3] = {0}; // 아날로그 축
    size_t axis; // long unsigned int 타입

    if (argc > 1) device = argv[1];
    else device = "/dev/input/js0"; // 패드 장치 디렉토리

    js = open(device, O_RDONLY); // 장치 읽어들이기(Read-Only)

    if (js == -1) perror("Could not open joystick"); // 패드 읽기 실패시

    while (readEvent(js, &event) == 0){
        switch (event.type){
            case JS_EVENT_BUTTON:
                printf("event.number: %u, event.value: %d\n", event.number, event.value);
                action(event);
                break;
            default: break;
        }    
        fflush(stdout);

        twist.linear.x = x * speed;
        twist.linear.y = y * speed;
        twist.linear.z = z * speed;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;

        pub.publish(twist); // 메시지 퍼블리싱
        ros::spinOnce();
    }

    close(js);
    return 0;
}