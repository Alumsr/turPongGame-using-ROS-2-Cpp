#include <functional>
#include <stdexcept>
#include <thread>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

#define KEYCODE_UP_R 0x41
#define KEYCODE_DOWN_R 0x42
#define KEYCODE_UP_L 0x73
#define KEYCODE_DOWN_L 0x77
#define KEYCODE_QUIT 0x51

bool running = true;

// 读取键盘输入
class KeyboardReader final {
  public: 
    KeyboardReader() {
      // 获取旧的控制台模式
      if (tcgetattr(0, & cooked_) < 0) {
        throw std::runtime_error("Failed to get old console mode");
      }
      // 复制旧的模式
      struct termios raw;
      memcpy( & raw, & cooked_, sizeof(struct termios));
      // 设置新的模式
      raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VEOL] = 1;
      raw.c_cc[VEOF] = 2;
      raw.c_cc[VTIME] = 1;
      raw.c_cc[VMIN] = 0;
      if (tcsetattr(0, TCSANOW, & raw) < 0) {
        throw std::runtime_error("Failed to set new console mode");
      }
    }

    // 读取一个字符
    char readOne() {
      char c = 0;
      int rc = read(0, & c, 1);
      if (rc < 0) {
        throw std::runtime_error("read failed");
      }

      return c;
    }

    // 析构函数，恢复旧的控制台模式
    ~KeyboardReader() {
      tcsetattr(0, TCSANOW, & cooked_);
    }

  private: 
    struct termios cooked_;
};

// 发布移动方向消息
class MoveTurtle {
  public: 
    MoveTurtle() {
      // 声明ROS节点
      nh_ = rclcpp::Node::make_shared("move_turtle");

      // 声明发布者
      publisherL = nh_ -> create_publisher <std_msgs::msg::Int8> ("move_left_turtle", 10);
      publisherR = nh_ -> create_publisher <std_msgs::msg::Int8> ("move_right_turtle", 10);
      // publisherS = nh_ -> create_publisher <std_msgs::msg::Int8> ("status", 10);
    }

    // 读取键盘输入并发布移动方向信息
    int keyLoop() {
      char c;

      // 创建新线程，spin在新线程中运行
      std::thread {
        std::bind( & MoveTurtle::spin, this)
      }.detach();

      // 输出提示信息
      printf("-----------------------------------------------------------------\n");
      printf("<UP>/<DOWN>\t|moving right turtle\n<w>/<s>\t|moving left turtle\n"/*<q>\t|quit\n"*/);
      printf("-----------------------------------------------------------------\n");

      while (running) {
        try {
          c = input_.readOne();
        } catch (const std::runtime_error & ) {
          perror("read():");
          return -1;
        }

        // // 输出读取到的字符
        // RCLCPP_DEBUG(nh_ -> get_logger(), "value: 0x%02X\n", c);

        // 根据读取到的字符，设置移动方向
        switch (c) {
        case KEYCODE_UP_R:
          // RCLCPP_DEBUG(nh_ -> get_logger(), "UP");
          left_direction = 1;
          right_direction = 0;
          break;
        case KEYCODE_DOWN_R:
          // RCLCPP_DEBUG(nh_ -> get_logger(), "DOWN");
          left_direction = -1;
          right_direction = 0;
          break;
        case KEYCODE_UP_L:
          // RCLCPP_DEBUG(nh_ -> get_logger(), "UP");
          left_direction = 0;
          right_direction = 1;
          break;
        case KEYCODE_DOWN_L:
          // RCLCPP_DEBUG(nh_ -> get_logger(), "DOWN");
          left_direction = 0;
          right_direction = -1;
          break;
        // case KEYCODE_QUIT: --待完成
        //   // RCLCPP_DEBUG(nh_ -> get_logger(), "QUIT");
        //   left_direction = 0;
        //   right_direction = 0;
        //   status = 1;
        //   running = false;
        //   break;
        default:
          left_direction = 0;
          right_direction = 0;
          break;
        }

        // 发布移动方向
        auto msg = std_msgs::msg::Int8();  // 创建消息
        msg.data = right_direction;
        publisherL -> publish(msg);        // 发布消息
        msg.data = left_direction;
        publisherR -> publish(msg);        // 发布消息

        // // 发布状态--待完成
        // auto msg_s = std_msgs::msg::Int8();  // 创建消息
        // msg_s.data = status;
        // publisherS -> publish(msg_s);        // 发布消息
      }

      return 0;
    }

  private:
    
    // 移动方向
    int right_direction = 0;
    int left_direction = 0;
    // int status = 0;

    // ROS节点的spin函数
    void spin() {
      rclcpp::spin(nh_);
    }

    // 声明节点和发布者
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher <std_msgs::msg::Int8> ::SharedPtr publisherL;
    rclcpp::Publisher <std_msgs::msg::Int8> ::SharedPtr publisherR;
    // rclcpp::Publisher <std_msgs::msg::Int8> ::SharedPtr publisherS;

    // 读键盘器
    KeyboardReader input_;
};

// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
void quit(int sig) {
  (void) sig;
  running = false;
}


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // 烫烫烫烫烫
  signal(SIGINT, quit);

  // 创建MoveTurtle对象
  MoveTurtle move_right_turtle;

  // 进入键盘输入循环
  int rc = move_right_turtle.keyLoop();

  // 关闭ROS节点
  rclcpp::shutdown();

  return rc;
}
