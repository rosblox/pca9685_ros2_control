#include <pca9685_hardware_interface/pca9685_comm.h>

#include <chrono>
#include <functional>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <ncurses.h>

class PwmTest : public rclcpp::Node
{
public:
  PwmTest() : Node("pwm_test_node")
  {
    this->declare_parameter("i2c_device", "/dev/i2c-1");
    this->declare_parameter("i2c_address", 0x40);
    this->declare_parameter("pwm_frequency", 50.0);
    this->declare_parameter("channel", 0);

    std::string i2c_device = this->get_parameter("i2c_device").as_string();
    int i2c_address        = this->get_parameter("i2c_address").as_int();
    double pwm_frequency   = this->get_parameter("pwm_frequency").as_double();
    channel_               = this->get_parameter("channel").as_int();

    pca = PiPCA9685::PCA9685(i2c_device, i2c_address);
    pca.set_pwm_freq(pwm_frequency);
    pca.set_pwm_ms(channel_, pulse_width_ms_);

    initscr();
    keypad(stdscr, TRUE);
    noecho();
    printw("Use the arrow keys to increase/decrease pulse width\nBe careful not to exceed your servo's physical limits\n");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PwmTest::timer_callback, this));
  }

  void timer_callback()
  {
    mvprintw(3, 0, "Pulse width (ms): %.2f", pulse_width_ms_);
    refresh();
    int ch = getch();
    switch (ch) {
        case 'q':
            rclcpp::shutdown();
            break;
        case KEY_UP:
            pulse_width_ms_ += 0.01;
            break;
        case KEY_DOWN:
            pulse_width_ms_ = std::max(0.0, pulse_width_ms_ - 0.01);
            break;
    }
    pca.set_pwm_ms(channel_, pulse_width_ms_);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  double pulse_width_ms_ = 1.5;
  PiPCA9685::PCA9685 pca;
  int channel_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PwmTest>());
  rclcpp::shutdown();
  endwin();
  return 0;
}