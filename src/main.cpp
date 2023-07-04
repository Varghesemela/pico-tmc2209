// #define PICO_FLASH_SPI_CLKDIV   4

#include <cmath>
#include <cstring>
#include <string>
using namespace std;

#include <pico/multicore.h>
#include <pico/sem.h>

#include "functions.h"
#include "hardware/adc.h"
#include "pico/printf.h"
#include "pico/unique_id.h"
#include "pins_pico_board.h"
#include "project_config.h"
#include "tmc2209.h"
pico_unique_board_id_t id_out;

struct repeating_timer timer1;
// TMC2209 Driver Init
tmc2209 driver(TMC_UART, 115200, 0b11, TMC_UART_RX, TMC_UART_TX, TMC_VREF, TMC_INDEX);


void GPIO_init() {
  gpio_init(LED_STATUS);
  gpio_set_dir(LED_STATUS, GPIO_OUT);
  gpio_put(LED_STATUS, LOW);

  gpio_init(TMC_DIRECTION);
  gpio_set_dir(TMC_DIRECTION, GPIO_OUT);
  gpio_put(TMC_DIRECTION, LOW);

  gpio_init(TMC_STEP);
  gpio_set_function(TMC_STEP, GPIO_FUNC_PWM);

  gpio_init(TMC_ENABLE);
  gpio_set_dir(TMC_ENABLE, GPIO_OUT);
  gpio_put(TMC_ENABLE, LOW);

}

#define BUFFER_LENGTH 50
char buffer[BUFFER_LENGTH];
const char s[2] = ",";
const char start_of_frame = '<', end_of_frame = '>';
string command_str, subcommand_str;
string data_str;
uint16_t buffer_index = 0, EOF_index = 0;
bool SOF_flag = false, EOF_flag = false;

union {
  uint8_t axis_val[4];
  float axis_fval;
} buffer_union;

// Core 1 interrupt Handler
void core1_interrupt_handler() {
  // Receive Raw Value, Convert and Print Temperature Value
  while (multicore_fifo_rvalid()) {
    printf("Core 1");
  }
  multicore_fifo_clear_irq();  // Clear interrupt
}

/**
 * \brief Core 1 main loop
 *
 * This function serves as the main loop for the secondary core when initialised
 * in the primary core.
 *
 */
[[noreturn]] void core1_entry() {
  // Configure Core 1 Interrupt
  multicore_fifo_clear_irq();
  multicore_lockout_victim_init();
  // irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_interrupt_handler);
  // irq_set_enabled(SIO_IRQ_PROC1, true);
  while (true) {
    gpio_put(LED_STATUS, true);
    sleep_ms(333);
    gpio_put(LED_STATUS, false);
    sleep_ms(333);
    // Just for fun, this core not being used
  }
}

bool get_block(struct repeating_timer *t) {
  // buffer_index = 0;
  // while(true){
  int c = getchar_timeout_us(0);
  // printf("%c\n", c);
  if (c != PICO_ERROR_TIMEOUT && buffer_index < BUFFER_LENGTH && !EOF_flag) {
    if (c == start_of_frame) {
      SOF_flag = true;
      command_str.clear();
      subcommand_str.clear();
      //            data_str.clear();
      std::fill(std::begin(data_str), std::end(data_str), '\0');
      std::fill(std::begin(buffer), std::end(buffer), '\0');
      buffer_index = 0;
    } else if (c == end_of_frame) {
      EOF_flag = true;
      EOF_index = buffer_index;

      command_str = strtok(buffer, s);
      if (command_str == "#RP64209") {
        reset_usb_boot(0, 0);
      }
      if (!is_only_alphabets(command_str)) {
        //                printf("Invalid command, change later\n");
      }

      // break;
    }
    if ((SOF_flag) && (!EOF_flag) && (c != start_of_frame)) {
      buffer[buffer_index++] = (c & 0xFF);
    }

  } else {
    // break;
  }
  // }
  return true;
}

int main() {
  //    set_sys_clock_khz(250000, true);
  stdio_init_all();

  GPIO_init();

  //    adc_init();
  //    adc_gpio_init(26);// Make sure GPIO is high-impedance, no pullups etc
  //    adc_select_input(0); //Or Select ADC input 1 (GPIO27)
  //    adc_set_clkdiv(25600);

  sleep_ms(2000);
  printf("Status: %d\n", add_repeating_timer_us(-10 * TICK_RATE, get_block, NULL, &timer1));

  // watchdog_enable(5000, 1);
  // while(!stdio_usb_connected());
  printf("Hello\n");

  uint32_t stepval = 0;
  //    multicore_launch_core1(core1_entry);
  gpio_put(TMC_ENABLE, LOW);
  driver.initialize();
  driver.set_current(1000);
  driver.initialize_stepcounter(&stepval);
  driver.set_microstepping(MICROSTEP_RESOLUTION_16);

  uint32_t base_frequency = 20;
  float uSTEP_FACTOR = 228.5714;
  while (true) {
    tight_loop_contents();
    //        can_loop();
    busy_wait_ms(100);
    if (!EOF_flag) {

    } else {
      if (command_str == "u2") {
        printf(BOARD_TYPE "\n");
        printf("ok\n");

      } else if (command_str == "TMC.status") {
        printf("\n\n");
        driver.check_driver_status();
        driver.check_motor_current();
        driver.check_open_load();
        driver.check_lowside_short();
        driver.check_ground_short();
        driver.check_OT_status();

      } else if (command_str == "uart") {
        printf("uart status %d\n", driver.check_uart_connection());
        printf("step value: %d", stepval);

      } else if (command_str == "TMC.microstep") {
        data_str = strtok(nullptr, s);
        printf(data_str.c_str(), "\n");

        uint microstep;
        microstep = (uint)strtod(data_str.c_str(), nullptr);
        driver.set_microstepping(microstep);
        printf("microstep set to %d\n", microstep);
        memset(buffer, '\0', sizeof(buffer));

      } else if (command_str == "on") {
        pwm_set_freq_duty(TMC_STEP, (uint32_t)(base_frequency * uSTEP_FACTOR), 50);
        pwm_set_enabled(pwm_gpio_to_slice_num(TMC_STEP), true);

      } else if (command_str == "speedon") {
        stepval = 0;
        driver.set_vactual((uint32_t)(base_frequency * uSTEP_FACTOR));

      } else if (command_str == "speedoff") {
        driver.set_vactual(0);

      } else if (command_str == "off") {
        pwm_set_enabled(pwm_gpio_to_slice_num(TMC_STEP), false);

      } else if (command_str == "fwd") {
        driver.set_motor_direction(true);

      } else if (command_str == "bck") {
        driver.set_motor_direction(false);

      } else if (command_str == "id") {
        pico_get_unique_board_id(
            &id_out);  // to receive the unique 64-bit chip ID for BOM tracking
        printf("Board ID: ");
        for (unsigned char i : id_out.id) {
          printf("%X", i);
        }
        printf("\n");

      } else if (command_str == "TMC.current") {
        // for IHOLD current control
        data_str = strtok(nullptr, s);
        printf(data_str.c_str(), "\n");

        uint current;
        current = (uint)strtod(data_str.c_str(), NULL);
        driver.set_current(current);
        printf("current set to %d\n", current);
        memset(buffer, '\0', sizeof(buffer));

      }
      buffer_index = 0;
      EOF_index = 0;
      memset(buffer, '\0', sizeof(buffer));
      SOF_flag = false;
      EOF_flag = false;
    }
  }
}
