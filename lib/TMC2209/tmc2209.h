//
// Created by sanjay on 25/5/23.
//

#ifndef TMC2209_H
#define TMC2209_H

#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#define R_SENSE          110  // mOhms
#define V_REF_MEASURED   2.29 // Volts

enum slave_address
{
    DRIVER_1,
    DRIVER_2,
    DRIVER_3,
    DRIVER_4
};

typedef enum
{
    MICROSTEP_RESOLUTION_128 = 1,
    MICROSTEP_RESOLUTION_64,
    MICROSTEP_RESOLUTION_32,
    MICROSTEP_RESOLUTION_16,
    MICROSTEP_RESOLUTION_8,
    MICROSTEP_RESOLUTION_4,
    MICROSTEP_RESOLUTION_2,
    MICROSTEP_RESOLUTION_FULLSTEP
}microstep_resolution;

typedef enum{
    NORMAL_OPERATION,
    FREEWHEELING,
    COIL_SHORTED_USING_LS_DRIVERS,
    COIL_SHORTED_USING_HS_DRIVERS
}standstill_mode;

typedef enum{
    TEMP_THRESHOLD_120_DEGREE,
    TEMP_THRESHOLD_143_DEGREE,
    TEMP_THRESHOLD_150_DEGREE,
    TEMP_THRESHOLD_157_DEGREE

}temperature_threshold;

enum current_factor{
    CURRENT_FACTOR_1_32 = 1,
    CURRENT_FACTOR_2_32,
    CURRENT_FACTOR_3_32,
    CURRENT_FACTOR_4_32,
    CURRENT_FACTOR_5_32,
    CURRENT_FACTOR_6_32,
    CURRENT_FACTOR_7_32,
    CURRENT_FACTOR_8_32,
    CURRENT_FACTOR_9_32,
    CURRENT_FACTOR_10_32,
    CURRENT_FACTOR_11_32,
    CURRENT_FACTOR_12_32,
    CURRENT_FACTOR_13_32,
    CURRENT_FACTOR_14_32,
    CURRENT_FACTOR_15_32,
    CURRENT_FACTOR_16_32,
    CURRENT_FACTOR_17_32,
    CURRENT_FACTOR_18_32,
    CURRENT_FACTOR_19_32,
    CURRENT_FACTOR_20_32,
    CURRENT_FACTOR_21_32,
    CURRENT_FACTOR_22_32,
    CURRENT_FACTOR_23_32,
    CURRENT_FACTOR_24_32,
    CURRENT_FACTOR_25_32,
    CURRENT_FACTOR_26_32,
    CURRENT_FACTOR_27_32,
    CURRENT_FACTOR_28_32,
    CURRENT_FACTOR_29_32,
    CURRENT_FACTOR_30_32,
    CURRENT_FACTOR_31_32,
    CURRENT_FACTOR_32_32

};

typedef struct regAddr{
    uint8_t GCONF = 0x00,
            GSTAT = 0x01,
            IFCNT = 0x02,
            SLAVECONF = 0x03,
            OTP_PROG = 0x04,
            OTP_READ = 0x05,
            IOIN = 0x06,
            FACTORY_CONF = 0x07,
            IHOLD_IRUN = 0x10,
            TPOWERDOWN = 0x11,
            TSTEP = 0x12,
            TPWMTHRS = 0x13,
            TCOOLTHRS = 0x14,
            VACTUAL = 0x22,
            SGTHRS = 0x40,
            SG_RESULT = 0x41,
            COOLCONF = 0x42,
            MSCNT = 0x6A,
            MSCURACT = 0x6B,
            CHOPCONF = 0x6C,
            DRVSTATUS = 0x6F,
            PWMCONF = 0x70,
            PWMSCALE = 0x71,
            PWM_AUTO = 0x72;
}regAddr_t;



class tmc2209
{
public:
    friend void step_pulse_callback();
    // Main routines
    tmc2209(uart_inst_t *uart, uint64_t baudrate, uint8_t slaveAddr, int rx_pin, int tx_pin, int vref_pin, int index_pin);
    int8_t initialize();
    int calculate_current_scale(uint16_t mA);

    // General Configuration routines
    void set_current_reference(bool value);
    void set_motor_direction(bool direction);
    void use_internal_rsense(bool value);
    void enable_spreadcycle(bool value);
    void set_pdn_pin_for_uart();
    void set_pdn_pin_for_current_scaling();
    void set_microstepping_from_mstep();
    void set_microstepping_from_ms1_ms2_pins();
    void filter_step_pulses(bool value);
    void set_general_configuration();

    uint32_t set_vref_voltage(double voltage);
    // Global Status routines

    // OTP write routines

    // OTP read routines

    // Read Input pins routines

    // Factory configuration routines

    // Velocity dependent control routines
    void set_current(uint16_t current_in_ma);
    void set_IHOLD_IRUN(uint8_t i_hold, uint8_t i_run, uint8_t i_hold_delay);
    void set_t_powerdown(uint8_t value);
    uint32_t read_t_step_value();
    void set_stealthchop_upper_limit_threshold(uint32_t value);
    void set_vactual(int32_t value);

    // Stallguard configuration routines
    void set_lower_limit_threshold_to_enable_coolstep(uint32_t value);
    void set_stallguard_detection_threshold_value(uint8_t value);
    uint16_t read_stallguard_value();

    // Coolstep configuration routines
    void set_coolstep_minimum_current(bool value); // 0->(0.5*I_RUN) ; 1->(0.25*I_RUN)
    void set_current_down_step_speed(uint8_t value); // value range = [0,3], Check datasheet
    void set_stallguard_hysteresis_value(uint8_t value); // Check datasheet
    void set_current_up_step_width(uint8_t value); // Check datasheet
    void set_minimum_stallguard_value_for_smart_current_control(uint8_t value); // Check Datsheet

    // Chopper configuration routines
    void enable_disable_low_side_short_protection(bool value);
    void enable_disable_short_to_ground_protection(bool value);
    void enable_double_edge_step_pulses(bool value);
    void set_interpolation_to_256_steps(bool value);
    void set_microstepping(uint8_t value);
    void set_vsense(bool value);
    void set_comparator_blank_time(uint8_t value);
    void set_hysteresis_offset_value(uint8_t value);
    void set_hysteresis_start_value(uint8_t value);
    void set_off_time_and_driver_disable(uint8_t value);

    // Driver Status routines
    int8_t check_uart_connection();
    int8_t check_driver_status();
    int8_t check_motor_current() const;
    int8_t check_open_load() const;
    int8_t check_lowside_short() const;
    int8_t check_ground_short() const;
    int8_t check_OT_status() const;


    // PWM configuration routines
    void set_pwm_standstill_mode(standstill_mode value);

    /* Dependencies */
    uint8_t calc_CRC8(uint8_t datagram[], uint8_t len);
    void write(uint8_t regAddress, uint32_t value);
    uint64_t sendDatagram(uint8_t* datagram, uint8_t len);
    uint32_t read(uint8_t regAddress);


    int8_t initialize_stepcounter(uint32_t *stepcounter);

private:
    /////////////////////////////////
    //Private variables declaration
    /////////////////////////////////
    uart_inst  *_uart_ID;
    uint8_t _tx_pin, _rx_pin, _vref_pin, _index_pin, _slaveaddr;
    uint32_t _baudrate;
    uint32_t *_stepcounter;
    uint8_t DATA_BITS = 8;
    uint8_t STOP_BITS = 1;
    uart_parity_t PARITY = UART_PARITY_NONE;
    uint8_t TMC2209_WRITE_BIT = 0x80;
    uint8_t TMC2209_READ_BIT = 0x00;
    uint8_t TMC2209_SYNC_BIT = 0x05;

    /////////////////////////////////
    // Private Function declarations
    /////////////////////////////////
    void initialize_uart();
    void initialize_gpio();

    /////////////////////////////////
    // Global Configuration registers
    /////////////////////////////////
    const static uint8_t ADDRESS_GCONF = 0x00;
    union GlobalConfig
    {
        struct
        {
            uint32_t i_scale_analog : 1;
            uint32_t internal_rsense : 1;
            uint32_t enable_spread_cycle : 1;
            uint32_t shaft : 1;
            uint32_t index_otpw : 1;
            uint32_t index_step : 1;
            uint32_t pdn_disable : 1;
            uint32_t mstep_reg_select : 1;
            uint32_t multistep_filt : 1;
            uint32_t test_mode : 1;
            uint32_t reserved : 22;
        };
        uint32_t bytes;
    };
    GlobalConfig global_config_{};

    /////////////////////////////////
    // Global Status registers
    /////////////////////////////////
    const static uint8_t ADDRESS_GSTAT = 0x01;
    union GlobalStatus
    {
        struct
        {
            uint32_t reset : 1;
            uint32_t drv_err : 1;
            uint32_t uv_cp : 1;
            uint32_t reserved : 29;
        };
        uint32_t bytes;
    };
    GlobalStatus global_status_{};

    /////////////////////////////////
    // Write Instruction Counter register
    /////////////////////////////////
    const static uint8_t ADDRESS_IFCNT = 0x02;

    /////////////////////////////////
    // Response Delay Time register
    /////////////////////////////////
    const static uint8_t ADDRESS_SENDDELAY = 0x03;
    union SendDelay
    {
        struct
        {
            uint32_t reserved_0 : 8;
            uint32_t senddelay : 8;
            uint32_t reserved_1 : 16;
        };
        uint32_t bytes;
    };

    /////////////////////////////////
    // OTP_WRITE and OTP_READ Registers
    /////////////////////////////////
    // Incomplete

    /////////////////////////////////
    // Read Pin Inputs register
    /////////////////////////////////
    const static uint8_t ADDRESS_IOIN = 0x06;
    union Input
    {
        struct
        {
            uint32_t enn : 1;
            uint32_t reserved_0 : 1;
            uint32_t ms1 : 1;
            uint32_t ms2 : 1;
            uint32_t diag : 1;
            uint32_t reserved_1 : 1;
            uint32_t pdn_serial : 1;
            uint32_t step : 1;
            uint32_t spread_en : 1;
            uint32_t dir : 1;
            uint32_t reserved_2 : 14;
            uint32_t version : 8;
        };
        uint32_t bytes;
    };
    const static uint8_t VERSION = 0x21;

    /////////////////////////////////
    // FACTORY CONFIG register
    /////////////////////////////////
    // Incomplete

    /* Velocity Dependent Driver Feature Control Register Set */
    /////////////////////////////////
    // I_HOLD_DELAY, I_HOLD & I_RUN (w) registers
    /////////////////////////////////
    const static uint8_t ADDRESS_IHOLD_IRUN = 0x10;
    union DriverCurrent
    {
        struct
        {
            uint32_t ihold : 5;
            uint32_t reserved_0 : 3;
            uint32_t irun : 5;
            uint32_t reserved_1 : 3;
            uint32_t iholddelay : 4;
            uint32_t reserved_2 : 12;
        };
        uint32_t bytes;
    };
    DriverCurrent driver_current_{};
    const static uint8_t PERCENT_MIN = 0;
    const static uint8_t PERCENT_MAX = 100;
    const static uint8_t CURRENT_SETTING_MIN = 0;
    const static uint8_t CURRENT_SETTING_MAX = 31;
    const static uint8_t HOLD_DELAY_MIN = 0;
    const static uint8_t HOLD_DELAY_MAX = 15;
    const static uint8_t IHOLD_DEFAULT = 16;
    const static uint8_t IRUN_DEFAULT = 31;
    const static uint8_t IHOLDDELAY_DEFAULT = 1;

    /////////////////////////////////
    //TPOWERDOWN(w) register - time from standstill detection to motor current power down
    /////////////////////////////////
    const static uint8_t ADDRESS_TPOWERDOWN = 0x11;
    const static uint8_t TPOWERDOWN_DEFAULT = 20;
    uint8_t tpowerdown{};

    /////////////////////////////////
    //TSTEP(r) register - Measured time between two step pulses
    /////////////////////////////////
    const static uint8_t ADDRESS_TSTEP = 0x12;
    uint32_t tstep{};

    /////////////////////////////////
    //TPWMTHRS(w) - Upper velocity limit to switch off StealthChop feature
    /////////////////////////////////
    const static uint8_t ADDRESS_TPWMTHRS = 0x13;
    const static uint32_t TPWMTHRS_DEFAULT = 0;
    uint32_t tpwmthrs{};

    /////////////////////////////////
    //VACTUAL(w) register - LOW->Take input from STEP ; HIGH->Set motor velocity and move using internal pulse generator
    /////////////////////////////////
    const static uint8_t ADDRESS_VACTUAL = 0x22;
    const static int32_t VACTUAL_DEFAULT = 0;
    const static int32_t VACTUAL_STEP_DIR_INTERFACE = 0;
    uint32_t vactual{};

    /* StallGuard Control Set registers */
    /////////////////////////////////
    // TCOOLTHRS(w) - Lower threshold velocity to switch ON COOLSTEP & STALLGUARD
    /////////////////////////////////
    const static uint8_t ADDRESS_TCOOLTHRS = 0x14;
    const static uint8_t TCOOLTHRS_DEFAULT = 0;
    uint32_t tcoolthrs{};

    /////////////////////////////////
    // SGTHRS(w) - Detection threshold for stall
    /////////////////////////////////
    const static uint8_t ADDRESS_SGTHRS = 0x40;
    const static uint8_t SGTHRS_DEFAULT = 0;
    uint8_t sgthrs{};

    /////////////////////////////////
    // SG_RESULT(r) - Stalling happens when SG_RESULT <= (2 * SGTHRS)
    /////////////////////////////////
    const static uint8_t ADDRESS_SG_RESULT = 0x41;
    uint16_t sg_result{};

    /////////////////////////////////
    // COOLSTEP register configuration (w)
    /////////////////////////////////
    const static uint8_t ADDRESS_COOLCONF = 0x42;
    const static uint8_t COOLCONF_DEFAULT = 0;
    union CoolConfig
    {
        struct
        {
            uint32_t semin : 4;
            uint32_t reserved_0 : 1;
            uint32_t seup : 2;
            uint32_t reserved_1 : 1;
            uint32_t semax : 4;
            uint32_t reserved_2 : 1;
            uint32_t sedn : 2;
            uint32_t seimin : 1;
            uint32_t reserved_3 : 16;
        };
        uint32_t bytes;
    };
    CoolConfig cool_config_{};
    bool cool_step_enabled_{};
    const static uint8_t SEIMIN_UPPER_CURRENT_LIMIT = 20;
    const static uint8_t SEIMIN_LOWER_SETTING = 0;
    const static uint8_t SEIMIN_UPPER_SETTING = 1;
    const static uint8_t SEMIN_OFF = 0;
    const static uint8_t SEMIN_MIN = 1;
    const static uint8_t SEMIN_MAX = 15;
    const static uint8_t SEMAX_MIN = 0;
    const static uint8_t SEMAX_MAX = 15;

    /* Microstepping Control Register Set (Sequencer Registers)*/
    /////////////////////////////////
    // MSCNT(r) - microstep counter
    /////////////////////////////////
    const static uint8_t ADDRESS_MSCNT = 0x6A;

    /////////////////////////////////
    // MSCURACT(r) - bits[0..8]->actual motor current for phase A ; bits[16..24]->actual motor current for phase B
    /////////////////////////////////
    const static uint8_t ADDRESS_MSCURACT = 0x6B;

    /* Chopper Control Regiters */
    /////////////////////////////////
    // CHOPCONF(r/w) - Chopper configuration
    /////////////////////////////////
    const static uint8_t ADDRESS_CHOPCONF = 0x6C;
    union ChopperConfig
    {
        struct
        {
            uint32_t toff : 4;
            uint32_t hstrt : 3;
            uint32_t hend : 4;
            uint32_t reserved_0 : 4;
            uint32_t tbl : 2;
            uint32_t vsense : 1;
            uint32_t reserved_1 : 6;
            uint32_t mres : 4;
            uint32_t interpolation : 1;
            uint32_t double_edge : 1;
            uint32_t diss2g : 1;
            uint32_t diss2vs : 1;
        };
        uint32_t bytes;
    };
    ChopperConfig chopper_config_{};
    const static uint32_t CHOPPER_CONFIG_DEFAULT = 0x10000053;
    const static uint8_t TBL_DEFAULT = 0b10;
    const static uint8_t HEND_DEFAULT = 0;
    const static uint8_t HSTART_DEFAULT = 5;
    const static uint8_t TOFF_DEFAULT = 3;
    const static uint8_t TOFF_DISABLE = 0;
    uint8_t toff_ = TOFF_DEFAULT;
    const static uint8_t MRES_256 = 0b0000;
    const static uint8_t MRES_128 = 0b0001;
    const static uint8_t MRES_064 = 0b0010;
    const static uint8_t MRES_032 = 0b0011;
    const static uint8_t MRES_016 = 0b0100;
    const static uint8_t MRES_008 = 0b0101;
    const static uint8_t MRES_004 = 0b0110;
    const static uint8_t MRES_002 = 0b0111;
    const static uint8_t MRES_001 = 0b1000;


    const static size_t MICROSTEPS_PER_STEP_MIN = 1;
    const static size_t MICROSTEPS_PER_STEP_MAX = 256;

    /////////////////////////////////
    // DRV_STATUS(r)
    /////////////////////////////////
    const static uint8_t ADDRESS_DRV_STATUS = 0x6F;
    struct Status
    {
        uint32_t over_temperature_warning : 1;
        uint32_t over_temperature_shutdown : 1;
        uint32_t short_to_ground_a : 1;
        uint32_t short_to_ground_b : 1;
        uint32_t low_side_short_a : 1;
        uint32_t low_side_short_b : 1;
        uint32_t open_load_a : 1;
        uint32_t open_load_b : 1;
        uint32_t over_temperature_120c : 1;
        uint32_t over_temperature_143c : 1;
        uint32_t over_temperature_150c : 1;
        uint32_t over_temperature_157c : 1;
        uint32_t reserved0 : 4;
        uint32_t current_scaling : 5;
        uint32_t reserved1 : 9;
        uint32_t stealth_chop_mode : 1;
        uint32_t standstill : 1;
    };
    Status driverstatus{};

    const static uint8_t CURRENT_SCALING_MAX = 31;

    union DriveStatus
    {
        struct
        {
            Status status;
        };
        uint32_t bytes;
    };

    /////////////////////////////////
    // PWMCONFIG(r/w)
    /////////////////////////////////
    const static uint8_t ADDRESS_PWMCONF = 0x70;
    union PwmConfig
    {
        struct
        {
            uint32_t pwm_offset : 8;
            uint32_t pwm_grad : 8;
            uint32_t pwm_freq : 2;
            uint32_t pwm_autoscale : 1;
            uint32_t pwm_autograd : 1;
            uint32_t freewheel : 2;
            uint32_t reserved : 2;
            uint32_t pwm_reg : 4;
            uint32_t pwm_lim : 4;
        };
        uint32_t bytes;
    };
    PwmConfig pwm_config_{};
    const static uint32_t PWM_CONFIG_DEFAULT = 0xC10D0024;
    const static uint8_t PWM_OFFSET_MIN = 0;
    const static uint8_t PWM_OFFSET_MAX = 255;
    const static uint8_t PWM_OFFSET_DEFAULT = 0x24;
    const static uint8_t PWM_GRAD_MIN = 0;
    const static uint8_t PWM_GRAD_MAX = 255;
    const static uint8_t PWM_GRAD_DEFAULT = 0x14;

    /////////////////////////////////
    // PWM_SCALE(r)
    /////////////////////////////////
    union PwmScale
    {
        struct
        {
            uint32_t pwm_scale_sum : 8;
            uint32_t reserved_0 : 8;
            uint32_t pwm_scale_auto : 9;
            uint32_t reserved_1 : 7;
        };
        uint32_t bytes;
    };
    const static uint8_t ADDRESS_PWM_SCALE = 0x71;

    /////////////////////////////////
    // PWM_AUTO(r)
    /////////////////////////////////
    union PwmAuto
    {
        struct
        {
            uint32_t pwm_offset_auto : 8;
            uint32_t reserved_0 : 8;
            uint32_t pwm_gradient_auto : 8;
            uint32_t reserved_1 : 8;
        };
        uint32_t bytes;
    };
    const static uint8_t ADDRESS_PWM_AUTO = 0x72;

};

#endif //TMC2209_H
