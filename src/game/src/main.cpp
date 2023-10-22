
/**
 * -----------------------------------
 * @brief main.cpp 结构一览
 * -----------------------------------
 * 定义: Game 类 继承自 Node:
 *    public:
 *        定义: 构造函数:
 *            操作: 乌龟初始化
 *            定义: 订阅者/发布者/客户端
 *            定义: 定时器(含回调)      ------>(2)
 *            操作: 日志输出 
 *    private:
 *        声明: 游戏中变量:
 *            声明: BALL速度和位置
 *            声明: LEFT和RIGHT速度
 *            定义: 得分
 *        声明: 订阅者/发布者/客户端
 *        定义: 订阅者回调函数
 *        定义: 订阅请求函数
 *        定义: 游戏主循环函数          ------>(3)
 *            操作: 计算BALL与乌龟距离
 *            更新: LEFT和RIGHT速度
 *            更新: BALL速度和位置
 *            更新: 得分
 *            更新: 重置BALL
 * 定义: main 函数:  
 *     操作: spin game                 ------>(1)
 * -----------------------------------
*/

#include <cstdlib>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8.hpp"


// 游戏类, 继承自rclcpp::Node
class Game : public rclcpp::Node {
  public:
    Game() : Node("game") {
      // 随机生成 BALL 的 y 轴速度
      std::srand(static_cast<unsigned int>(std::time(nullptr)));
      vel_y = (std::rand() % 401 - 200) / 100;


      /**
       * @brief Request Publishing: Turtles Initialization
      */


      // 发送服务请求: 生成乌龟

      spawn_cli = this->create_client<turtlesim::srv::Spawn>("spawn");
      while(!spawn_cli->wait_for_service(std::chrono::seconds(1))) {  // 等待服务可用
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      send_request_spawn_left();     // LEFT
      send_request_spawn_right();    // RIGHT


      // 发送服务请求: 关闭画笔

      set_pen_ball_cli = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
      while(!set_pen_ball_cli->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      set_pen_left_cli = this->create_client<turtlesim::srv::SetPen>("left/set_pen");
      while(!set_pen_left_cli->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      set_pen_right_cli = this->create_client<turtlesim::srv::SetPen>("right/set_pen");
      while(!set_pen_right_cli->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      send_set_pen_ball_request();
      send_set_pen_left_request();
      send_set_pen_right_request();


      /**
       * @brief ROS 2 Members Definition
      */


      // 定义客户端: 重置 BALL 位置

      teleport_cli = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");

      while(!teleport_cli->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }


      // 定义订阅者: 订阅乌龟位置
      // !! 乌龟位置由 turtlesim_node 通过 /turtleName/pose    话题发布

      ball_pose_sub = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&Game::ball_pose_sub_callback, this, std::placeholders::_1));
      left_pose_sub = this->create_subscription<turtlesim::msg::Pose>("left/pose", 10, std::bind(&Game::left_pose_sub_callback, this, std::placeholders::_1));
      right_pose_sub = this->create_subscription<turtlesim::msg::Pose>("right/pose", 10, std::bind(&Game::right_pose_sub_callback, this, std::placeholders::_1));
    

      // 定义发布者: 发布乌龟速度
      // !! 乌龟速度由 turtlesim_node 通过 /turtleName/cmd_vel 话题订阅

      ball_pose_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
      left_pose_pub = this->create_publisher<geometry_msgs::msg::Twist>("left/cmd_vel", 10);
      right_pose_pub = this->create_publisher<geometry_msgs::msg::Twist>("right/cmd_vel", 10);


      // 定义订阅者: 订阅玩家输入(移动方向: 1上/-1下)
      move_left_turtle_sub = this->create_subscription<std_msgs::msg::Int8>("move_left_turtle", 10, std::bind(&Game::move_left_sub_callback, this, std::placeholders::_1));
      move_right_sub = this->create_subscription<std_msgs::msg::Int8>("move_right_turtle", 10, std::bind(&Game::move_right_sub_callback, this, std::placeholders::_1));

      // 定义订阅者: 订阅游戏状态(0开始/1结束)
      // !! 游戏状态由 nh_ 通过 /status 话题发布
      // status_sub = this->create_subscription<std_msgs::msg::Int8>("status", 10, [this](const std_msgs::msg::Int8::SharedPtr msg) {status = msg->data;});


      /**
       * @brief Main
      */


      // 定时器: 游戏循环主体, 绑定回调函数 game_loop; 
      timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Game::game_loop, this));
      

      // 输出提示
      RCLCPP_INFO(this->get_logger(), "-------------------Game Start-------------------");
    }

  private:
    /**
     * @brief 1. 游戏变量声明
    */

    // BALL的速度
    double vel_x = -2.0;
    double vel_y;

    // BALL的位置
    double tmp_x;
    double tmp_y;

    // 得分
    int left_score = 0;
    int right_score = 0;

    // LEFT和RIGHT移动方向
    int directionR;
    int directionL;


    /**
     * @brief 2. ROS2相关对象声明
    */

    // 定时器: 游戏循环主体
    rclcpp::TimerBase::SharedPtr timer;
    
    // 消息: 乌龟位置
    turtlesim::msg::Pose ball_pose = turtlesim::msg::Pose(); 
    turtlesim::msg::Pose left_pose = turtlesim::msg::Pose(); 
    turtlesim::msg::Pose right_pose = turtlesim::msg::Pose(); 

    // 客户端: 生成乌龟
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_cli;

    // 客户端: 关闭画笔
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_ball_cli;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_left_cli;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_right_cli;

    // 客户端: 重置BALL的坐标
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_cli;

    // 订阅者: 订阅乌龟位置
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr ball_pose_sub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr left_pose_sub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr right_pose_sub;
    
    // 发布者: 发布乌龟位置
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ball_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr left_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr right_pose_pub;

    // 订阅者: 订阅玩家输入
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr move_left_turtle_sub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr move_right_sub;

    // // 订阅者: 订阅游戏状态 --待完成
    // rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr status_sub;


    /**
     * @brief 3. 函数定义
    */

    // 游戏循环主体函数
    void game_loop() {
      // // 游戏状态检测 --待完成
      // if(status == 1) {
      //   RCLCPP_INFO(this->get_logger(), "-------------------FINAL RESULT-------------------");
      //   RCLCPP_INFO(this->get_logger(), "LEFT %d : %d RIGHT", left_score, right_score);
      //   RCLCPP_INFO(this->get_logger(), "-------------------GAME OVER-------------------");
      //   if(left_score > right_score) {
      //     RCLCPP_INFO(this->get_logger(), "-------------------LEFT WIN-------------------");
      //   } else if(left_score < right_score) {
      //     RCLCPP_INFO(this->get_logger(), "-------------------RIGHT WIN-------------------");
      //   } else {
      //     RCLCPP_INFO(this->get_logger(), "-------------------DRAW-------------------");
      //   }
      //   return;
      // }


      // 计算BALL与乌龟距离
      double dis_left = sqrt(pow(ball_pose.x - left_pose.x, 2) + pow(ball_pose.y - left_pose.y, 2));
      double dis_right = sqrt(pow(ball_pose.x - right_pose.x, 2) + pow(ball_pose.y - right_pose.y, 2));


      // LEFT和RIGHT速度发布: 根据按键检测

      auto left_msg = geometry_msgs::msg::Twist();    // 定义消息
      if(directionL == 0) {
        left_msg.linear.y = 0.0;
      } else if(directionL == -1) {
        left_msg.linear.y = 2.0;
      } else if(directionL == 1) {
        left_msg.linear.y = -2.0;
      }
      if(left_pose.y > 10.5 && left_msg.linear.y == -2.0) {
        left_msg.linear.y = 0.0;
      } else if(left_pose.y < 0.5 && left_msg.linear.y == 2.0) {
        left_msg.linear.y = 0.0;
      }
      left_pose_pub->publish(left_msg);             // 发布消息

      auto right_msg = geometry_msgs::msg::Twist();    // 定义消息
      if(directionR == 0) {
        right_msg.linear.y = 0.0;
      } else if(directionR == -1) {
        right_msg.linear.y = 2.0;
      } else if(directionR == 1) {
        right_msg.linear.y = -2.0;
      }
      if(right_pose.y > 10.5 && right_msg.linear.y == -2.0) {
        right_msg.linear.y = 0.0;
      } else if(right_pose.y < 0.5 && right_msg.linear.y == 2.0) {
        right_msg.linear.y = 0.0;
      }
      right_pose_pub->publish(right_msg);             // 发布消息



      // BALL速度赋值: 根据碰撞检测

      if(dis_left < 0.5 && left_pose.x < ball_pose.x) {           // 撞到LEFT且在LEFT右侧
        // 根据LEFT速度调整BALL速度
        if(left_msg.linear.y == 2.0 && vel_y <= 1.5) {
          vel_y += 0.5;
        } else if(left_msg.linear.y == -2.0 && vel_y >= -1.5) {   
          vel_y -= 0.5;
        }
        // BALL x轴速度反向
        vel_x = abs(vel_x);
      } else if(dis_right < 0.5 && right_pose.x > ball_pose.x) {  // 撞到RIGHT且在RIGHT左侧
        // 根据RIGHT速度调整BALL速度
        if (right_msg.linear.y == -2.0 && vel_y <= 1.5) {
          vel_y += 0.5;
        } else if (right_msg.linear.y == 2.0 && vel_y >= -1.5) {
          vel_y -= 0.5;
        }
        // BALL x轴速度反向
        vel_x = -abs(vel_x);
      }


      // BALL速度赋值: 碰壁反弹后

      if(ball_pose.y > 10.5) {        // 碰上壁
        vel_y = -abs(vel_y);
      } else if(ball_pose.y < 1.0) {  // 碰下壁
        vel_y = abs(vel_y);
      }


      // 得分赋值 & 重置请求发送: 碰壁结束后

      if(ball_pose.x > 10.5) {                                                      // 碰右壁
        send_teleport_turtle_request();

        left_score++;
        vel_x += 0.1;  // 防止卡位

        RCLCPP_INFO(this->get_logger(), "-------------------------------");
        RCLCPP_INFO(this->get_logger(), "LEFT scored!");
        RCLCPP_INFO(this->get_logger(), "LEFT %d : %d RIGHT", left_score, right_score);        
      } else if(ball_pose.x < 0.5 && (ball_pose.x != 0.0 && ball_pose.y != 0.0)) {  // 碰左壁且BALL位置已初始化
        send_teleport_turtle_request();

        right_score++;
        vel_x -= 0.1;  // 防止卡位

        RCLCPP_INFO(this->get_logger(), "-------------------------------");
        RCLCPP_INFO(this->get_logger(), "RIGHT scored!");
        RCLCPP_INFO(this->get_logger(), "LEFT %d : %d RIGHT", left_score, right_score);
      }


      // BALL速度发布

      auto ball_msg = geometry_msgs::msg::Twist();
      ball_msg.linear.x = vel_x;
      ball_msg.linear.y = vel_y;
      ball_pose_pub->publish(ball_msg);
    }  // game_loop


    // 请求发送函数: 生成 2 乌龟(默认的turtle1作为BALL)

    void send_request_spawn_left() {
      auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
      
      request->x = 0.5;
      request->y = 5.5;
      request->theta = 0.0;
      request->name = "left";

      spawn_cli->async_send_request(request);
    }

    void send_request_spawn_right() {
      auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
      
      request->x = 10.5;
      request->y = 5.5;
      request->theta = 3.1415926;
      request->name = "right";

      spawn_cli->async_send_request(request);
    }


    // 请求发送函数: 关闭乌龟画笔

    void send_set_pen_ball_request() {
      auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
      request->off = 1;
      set_pen_ball_cli->async_send_request(request);
    }

    void send_set_pen_left_request() {
      auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
      request->off = 1;
      set_pen_left_cli->async_send_request(request);
    }

    void send_set_pen_right_request() {
      auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
      request->off = 1;
      set_pen_right_cli->async_send_request(request);
    }


    // 请求发送函数: 重置龟速和BALL的位置
    void send_teleport_turtle_request() {
      auto ball_msg = geometry_msgs::msg::Twist();
      ball_msg.linear.x = 0.0;
      ball_msg.linear.y = 0.0;
      ball_pose_pub->publish(ball_msg);

      auto left_msg = geometry_msgs::msg::Twist();
      left_msg.linear.y = 0.0;
      left_pose_pub->publish(left_msg);

      auto right_msg = geometry_msgs::msg::Twist();
      right_msg.linear.y = 0.0;
      right_pose_pub->publish(right_msg);

      auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
      
      request->x = 5.5;
      request->y = 5.5;

      teleport_cli->async_send_request(request);

      ball_pose.x = 5.5;
      ball_pose.y = 5.5;
    }


    // 订阅者回调函数: 给乌龟位置 pose 赋值

    void ball_pose_sub_callback(const turtlesim::msg::Pose & msg) {
      ball_pose = msg;
    }

    void left_pose_sub_callback(const turtlesim::msg::Pose & msg) {
      left_pose = msg;
    }

    void right_pose_sub_callback(const turtlesim::msg::Pose & msg) {
      right_pose = msg;
    }


    // 订阅者回调函数: 给乌龟速度(移动方向) direction 赋值
    void move_right_sub_callback(const std_msgs::msg::Int8 & msg) {
      directionR = msg.data;
    }

    void move_left_sub_callback(const std_msgs::msg::Int8 & msg) {
      directionL = msg.data;
    }
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Game>());
  rclcpp::shutdown();
  return 0;
}

