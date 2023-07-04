//
// Created by sanjay on 25/5/23.
//

#include "tmc2209.h"

static tmc2209 *instance = nullptr;

////////////////////////////////////////////////
/* TOP level Driver Routines*/
////////////////////////////////////////////////

// Driver Pin Initialisation
tmc2209 :: tmc2209(uart_inst_t *uart, uint64_t baudrate, uint8_t slaveAddr, int rx_pin, int tx_pin, int vref_pin, int index_pin)
        : _uart_ID(uart), _tx_pin(tx_pin), _rx_pin(rx_pin), _vref_pin(vref_pin), _index_pin(index_pin), _slaveaddr(slaveAddr), _baudrate(baudrate){
    instance = this;
}

// Driver Initialisation
int8_t tmc2209::initialize(){
    initialize_gpio();
    initialize_uart();

    set_general_configuration();
    set_interpolation_to_256_steps(false);

    return check_uart_connection();
}

////////////////////////////////////////////////
/* Global Configuration Registers Write routines */
////////////////////////////////////////////////

// Index PIN configuration routines not defined

void tmc2209::set_current_reference(bool value){
    // true:
    global_config_.i_scale_analog = value;
    write(ADDRESS_GCONF, global_config_.bytes);
}

void tmc2209::use_internal_rsense(bool value){
    global_config_.internal_rsense = value;
    write(ADDRESS_GCONF, global_config_.bytes);
}

void tmc2209::enable_spreadcycle(bool value){
    global_config_.enable_spread_cycle = value;
    write(ADDRESS_GCONF, global_config_.bytes);
}

void tmc2209::set_motor_direction(bool direction){
    global_config_.shaft = direction;
    write(ADDRESS_GCONF, global_config_.bytes);
}

void tmc2209::set_pdn_pin_for_uart(){
    global_config_.pdn_disable = true;
    write(ADDRESS_GCONF, global_config_.bytes);
}

void tmc2209::set_pdn_pin_for_current_scaling(){
    global_config_.pdn_disable = false;
    write(ADDRESS_GCONF, global_config_.bytes);
}

void tmc2209::set_microstepping_from_ms1_ms2_pins(){
    global_config_.mstep_reg_select = false;
    write(ADDRESS_GCONF, global_config_.bytes);
}

void tmc2209::set_microstepping_from_mstep(){
    global_config_.mstep_reg_select = true;
    write(ADDRESS_GCONF, global_config_.bytes);
}

void tmc2209::filter_step_pulses(bool value){
    global_config_.multistep_filt = value;
    write(ADDRESS_GCONF, global_config_.bytes);
}

void tmc2209::set_general_configuration(){
    global_config_.i_scale_analog = true;
    global_config_.internal_rsense = false;
    global_config_.enable_spread_cycle = true;
    global_config_.shaft = false;
    global_config_.index_otpw = false;
    global_config_.index_step = true;
    global_config_.pdn_disable = true;
    global_config_.mstep_reg_select = true;
    global_config_.multistep_filt = false;
    global_config_.test_mode = false;

    write(ADDRESS_GCONF, global_config_.bytes);

    chopper_config_.bytes = read(ADDRESS_CHOPCONF);
}

////////////////////////////////////////////////
/* Global Status flags Read routines */
////////////////////////////////////////////////
int8_t tmc2209 :: check_uart_connection(){
    uint8_t uart_txcount;
    uart_txcount = (uint8_t)read(ADDRESS_IFCNT);
    uint32_t value_gcof = read(ADDRESS_GCONF);
    uint32_t uart_enable = 0x00000040;
    uart_enable |= value_gcof;
    write(ADDRESS_GCONF, uart_enable);

    // check UART working or not

    //TODO : check ADDRESS_IFCNT for transmission confirmation before and after any write
    /*This register becomes incremented with each successful UART interface write access.
     * Read out to check the serial transmission for lost data. */
    if((uint8_t)read(ADDRESS_IFCNT) == (uart_txcount+1)) {
        printf("*UART connection established succesfully!! *\n");
        return PICO_OK;
    } else {
        return PICO_ERROR_CONNECT_FAILED;
    }
}

int8_t tmc2209:: check_driver_status(){

    if(check_uart_connection() != PICO_OK) {
        printf("*UART CONNECTION NOT ESTABLISHED*\n");
        return PICO_ERROR_CONNECT_FAILED;
    }

    uint32_t value = read(ADDRESS_DRV_STATUS);
    driverstatus.over_temperature_warning = value & 0x01;
    driverstatus.over_temperature_shutdown = (value & 0x02)>>1;
    driverstatus.short_to_ground_a = (value & 0x04)>>2;
    driverstatus.short_to_ground_b = (value & 0x08)>>3;
    driverstatus.low_side_short_a = (value & 0x10)>>4;
    driverstatus.low_side_short_b = (value & 0x20)>>5;
    driverstatus.open_load_a = (value & 0x40)>>6;
    driverstatus.open_load_b = (value & 0x80)>>7;
    driverstatus.over_temperature_120c = (value & 0x100)>>8;
    driverstatus.over_temperature_143c = (value & 0x200)>>9;
    driverstatus.over_temperature_150c = (value & 0x400)>>10;
    driverstatus.over_temperature_157c = (value & 0x800)>>11;
    driverstatus.reserved0 = (value & 0xF000)>>12;
    driverstatus.current_scaling = (value & 0x1F0000)>>16;
    driverstatus.reserved1 = (value & 0x3FE00000)>>21;
    driverstatus.stealth_chop_mode = (value & 0x40000000)>>30;
    driverstatus.standstill = (value & 0x80000000)>>31;

    return PICO_OK;
}

int8_t tmc2209::check_motor_current() const {
    printf("actual current to motor: %d\n", driverstatus.current_scaling);

    return PICO_OK;
}

int8_t tmc2209::check_open_load() const{
    if(driverstatus.open_load_a || driverstatus.open_load_b){
        if (driverstatus.open_load_a) printf("open phase A\n ");
        if (driverstatus.open_load_b) printf("open phase B\n ");
        printf("The driver takes no action upon it. This is just an informative flag\n");
    }

    return PICO_OK;
}
int8_t tmc2209::check_lowside_short() const{
    if(driverstatus.low_side_short_a || driverstatus.low_side_short_b){
        if (driverstatus.low_side_short_a) printf("Low side Short on phase A\n ");
        if (driverstatus.low_side_short_b) printf("Low side Short on phase B\n ");
        printf("ALERT! Drivers disabled\n");
    }

    return PICO_OK;
}
int8_t tmc2209::check_ground_short() const{
    if(driverstatus.short_to_ground_a || driverstatus.short_to_ground_b){
        if (driverstatus.short_to_ground_a) printf("Short to GND on phase A\n ");
        if (driverstatus.short_to_ground_b) printf("Short to GND on phase B\n ");
        printf("ALERT! Driver disabled\n");
    }

    return PICO_OK;
}
int8_t tmc2209::check_OT_status() const{
    if(driverstatus.over_temperature_warning){
        printf("Over temperature warning!\n");
    }

    if(driverstatus.over_temperature_shutdown){
        printf("Temperature limit exceeded!\n");
        printf("ALERT! Drivers disabled\n");
    }

    return PICO_OK;
}


////////////////////////////////////////////////
/* OTP Programming Write routines */
////////////////////////////////////////////////

////////////////////////////////////////////////
/* OTP Read routines */
////////////////////////////////////////////////

////////////////////////////////////////////////
/* Read Pin Inputs routines */
////////////////////////////////////////////////

////////////////////////////////////////////////
/* Factory Configuration Read Write routines */
////////////////////////////////////////////////

////////////////////////////////////////////////
/* Velocity Dependent Control Configuration routines */
////////////////////////////////////////////////

void tmc2209::set_current(uint16_t current_in_mA){
    double vref_voltage = ((float)current_in_mA/2000)*2.4;   //Max vref value is 2.4V

    this->set_vref_voltage(vref_voltage);
    this->set_IHOLD_IRUN(uint8_t(30), 30, 1);
    write(ADDRESS_IHOLD_IRUN, driver_current_.bytes);
}

void tmc2209::set_IHOLD_IRUN(uint8_t i_hold, uint8_t i_run, uint8_t i_hold_delay){
    driver_current_.ihold = i_hold;
    driver_current_.irun = i_run;
    driver_current_.iholddelay = i_hold_delay;

    write(ADDRESS_IHOLD_IRUN, driver_current_.bytes);
}

void tmc2209::set_t_powerdown(uint8_t value){
    tpowerdown = value;

    if(tpowerdown > 20 || tpowerdown < 2)
        printf("STEPPER DRIVER ERROR:TPowerDown register not set. Value out of range!\n");
    else
        write(ADDRESS_TPOWERDOWN, tpowerdown);
}

uint32_t tmc2209::read_t_step_value(){
    tstep = read(ADDRESS_TSTEP);
    return tstep & 0xFFFFF; // Return 20-bit value
}

void tmc2209::set_stealthchop_upper_limit_threshold(uint32_t value){
    tpwmthrs = value;
    write(ADDRESS_TPWMTHRS, tpwmthrs);
}

/* This function is to set speed and move stepper via UART */
void tmc2209::set_vactual(int32_t value){
    vactual = value;
    write(ADDRESS_VACTUAL, vactual);
}

/* StallGuard cofiguration routines */
void tmc2209::set_lower_limit_threshold_to_enable_coolstep(uint32_t value){
    tcoolthrs = value;
    write(ADDRESS_TCOOLTHRS, tcoolthrs);
}

void tmc2209::set_stallguard_detection_threshold_value(uint8_t value){
    sgthrs = value;
    write(ADDRESS_SGTHRS, sgthrs);
}

uint16_t tmc2209::read_stallguard_value(){
    sg_result = read(ADDRESS_SG_RESULT);
    return sg_result;
}

// Coolstep current control
void tmc2209::set_coolstep_minimum_current(bool value){
    cool_config_.seimin = value;
    write(ADDRESS_COOLCONF, cool_config_.bytes);
}

void tmc2209::set_current_down_step_speed(uint8_t value){
    cool_config_.sedn = value;
    write(ADDRESS_COOLCONF, cool_config_.bytes);
}

void tmc2209::set_stallguard_hysteresis_value(uint8_t value){
    cool_config_.semax = value;
    write(ADDRESS_COOLCONF, cool_config_.bytes);
}

void tmc2209::set_current_up_step_width(uint8_t value){
    cool_config_.seup = value;
    write(ADDRESS_COOLCONF, cool_config_.bytes);
}

void tmc2209::set_minimum_stallguard_value_for_smart_current_control(uint8_t value){
    cool_config_.semin = value;
    write(ADDRESS_COOLCONF, cool_config_.bytes);
}

////////////////////////////////////////////////
/* Sequencer Register Routines */
////////////////////////////////////////////////

////////////////////////////////////////////////
/* Chopper Control Routines */
////////////////////////////////////////////////
// Chopper Configuration
void tmc2209::enable_disable_low_side_short_protection(bool value){
    chopper_config_.diss2vs = value;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

void tmc2209::enable_disable_short_to_ground_protection(bool value){
    chopper_config_.diss2g = value;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

void tmc2209::enable_double_edge_step_pulses(bool value){
    chopper_config_.double_edge = value;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

void tmc2209::set_interpolation_to_256_steps(bool value){
    chopper_config_.interpolation = value;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

void tmc2209::set_microstepping(uint8_t value){
    chopper_config_.mres = value;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

void tmc2209::set_vsense(bool value){
    chopper_config_.vsense = value;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

void tmc2209::set_comparator_blank_time(uint8_t value){
    chopper_config_.tbl = value;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

void tmc2209::set_hysteresis_offset_value(uint8_t value){
    chopper_config_.hend = value;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

void tmc2209::set_hysteresis_start_value(uint8_t value){
    chopper_config_.hstrt = value;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

void tmc2209::set_off_time_and_driver_disable(uint8_t value){
    chopper_config_.toff = value;
    write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

// Driver Status register routines

// PWM configuration routines - Stealthchop
void tmc2209::set_pwm_standstill_mode(standstill_mode value){
    pwm_config_.freewheel = value;
    write(ADDRESS_PWMCONF, pwm_config_.bytes);
}

// PWM Scale register routine - Stealthchop

// PWM Auto register routine - Stealthchop

////////////////////////////////////////////////
/* Dependency Routines START */
////////////////////////////////////////////////

// IRQ for counting step pulses
void step_pulse_callback(){
    *instance->_stepcounter+=1;
}
// Initialize UART Peripheral of RPI Pico
void tmc2209::initialize_uart(){
    uart_init(_uart_ID, _baudrate);
    uart_set_translate_crlf((uart_inst_t *) _uart_ID, false);
    uart_set_format(_uart_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(_uart_ID, true);
}

// Initialize Motor GPIO
void tmc2209::initialize_gpio(){
    // GPIO Initalizations
    gpio_set_function(_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(_rx_pin, GPIO_FUNC_UART);
    if(_vref_pin >= 0){
        gpio_set_function(_vref_pin, GPIO_FUNC_PWM);
        pwm_set_clkdiv(pwm_gpio_to_slice_num(_vref_pin), ((float)clock_get_hz(clk_sys)/50)); // pwm clock should now be running at 1MHz

    }
    if(_index_pin >= 0){
        gpio_init(_index_pin);
        gpio_set_dir(_index_pin, GPIO_IN);

    }

}

int8_t tmc2209::initialize_stepcounter(uint32_t *stepcounter) {
    if(_index_pin >= 0) {
        this->_stepcounter = stepcounter;
        gpio_set_irq_enabled_with_callback(this->_index_pin, GPIO_IRQ_EDGE_RISE, true,
                                           reinterpret_cast<gpio_irq_callback_t>(&step_pulse_callback));
    }
    else {
        return -1;
    }
    return 0;
}

// Calculate CRC
uint8_t tmc2209 :: calc_CRC8(uint8_t datagram[], uint8_t len){
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t currentByte = datagram[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            crc &= 0xff;
            currentByte = currentByte >> 1;
        }
    }
    return crc;
}

void tmc2209 :: write(uint8_t regAddress, uint32_t value){
    uint8_t len = 7;
    uint8_t retData[8];
    uint64_t out = 0;
    regAddress |= TMC2209_WRITE_BIT;
    uint8_t datagram[] = {TMC2209_SYNC_BIT, _slaveaddr, regAddress, (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)(value >> 0), 0x00};

    datagram[len] = calc_CRC8(datagram, len);

    for(uint8_t i=0; i<=len; i++) {
        uart_putc(_uart_ID, datagram[i]);
    }
    busy_wait_ms(3);

}


uint64_t tmc2209 :: sendDatagram(uint8_t* datagram, const uint8_t len){
    for (int i = 0; i <= len; i++) {
        uart_putc(_uart_ID, datagram[i]);
    }

    busy_wait_ms(5);

    // scan for the rx frame and read it
    uint32_t sync_target = (static_cast<uint32_t>(datagram[0]) << 16) | 0xFF00 | datagram[2];
    uint32_t sync = 0;

    do {

        uint8_t res;
        if(uart_is_readable_within_us(_uart_ID, 10)){
            uart_read_blocking(_uart_ID, &res, 1);
        } else {
            return PICO_ERROR_TIMEOUT;
        }

        if (res < 0) continue;

        sync <<= 8;
        sync |= res & 0xFF;
        sync &= 0xFFFFFF;

    } while (sync != sync_target);

    uint64_t out = sync;

    for (uint8_t i = 0; i < 5;)
    {

        uint8_t res;
        if(uart_is_readable_within_us(_uart_ID, 10))
        {
            uart_read_blocking(_uart_ID, &res, 1);
        }
        else return 0;
        if (res < 0) continue;

        out <<= 8;
        out |= res & 0xFF;

        i++;
    }
    return out;
}


uint32_t tmc2209 :: read(uint8_t regAddress){
    /*constexpr*/ uint8_t len = 3;
    regAddress |= TMC2209_READ_BIT;
    uint8_t datagram[] = {TMC2209_SYNC_BIT, _slaveaddr, regAddress, 0x00};
    datagram[len] = calc_CRC8(datagram, len);
    uint64_t out = 0x00000000UL;
    for (uint8_t i = 0; i < 2; i++){
        out = sendDatagram(datagram, len);

        busy_wait_ms(3);

        uint8_t out_datagram[] = {
                static_cast<uint8_t>(out>>56),
                static_cast<uint8_t>(out>>48),
                static_cast<uint8_t>(out>>40),
                static_cast<uint8_t>(out>>32),
                static_cast<uint8_t>(out>>24),
                static_cast<uint8_t>(out>>16),
                static_cast<uint8_t>(out>> 8),
                static_cast<uint8_t>(out>> 0)
        };
        uint8_t crc = calc_CRC8(out_datagram, 7);
        if ((crc != static_cast<uint8_t>(out)) || crc == 0 ) {
            out = 0x00;
        } else {
            break;
        }
    }

    // printf("\nReceived Data:%lx\n", (uint32_t)out>>8);
    return out>>8;
}

int tmc2209 ::calculate_current_scale(uint16_t mA){
    // To add if(v_sense_enabled) and else
    // long int CS = mapRange(mA, 0, 100, 1, 31);
    // printf("CS Value: %ld\n", CS);

    int32_t CS;
    CS = (int32_t)((32.0 * 1.41421 * (mA / 1000.0) * (R_SENSE + 20) / /*0.325*/297.7) - 1);
    if(CS < 16){
        set_vsense(true);
        CS = (int32_t)(((32.0 * 1.41421 * (mA / 1000.0) * (R_SENSE + 20)) / /*0.180*/164.88) - 1);
    } else {
        set_vsense(false);
    }
    if (CS > 31) {
        CS = 31;
    }

    return CS;
}

uint32_t tmc2209::set_vref_voltage(double voltage){
    if (voltage > 2.4){     //Max vref value for TMC2209 is 2.4V
        voltage = 2.4;
    }
    double duty_cycle = (voltage*100)/3.3;
    uint16_t  frequency = 2*20*1000;    //20KHz

    uint32_t clock = clock_get_hz(clk_sys);
    uint32_t divider16 = (clock / frequency) / 4096 +
                         (clock % (frequency * 4096) != 0);
    if (divider16 / 16 == 0) {
        divider16 = 16;
    }

    uint32_t wrap = (((clock * 16) / divider16) / frequency) - 1;
    pwm_set_clkdiv_int_frac(pwm_gpio_to_slice_num(_vref_pin), divider16 / 16,
                            divider16 & 0xF);
    pwm_set_phase_correct(pwm_gpio_to_slice_num(_vref_pin), true);
    pwm_set_wrap(pwm_gpio_to_slice_num(_vref_pin), wrap);
    pwm_set_chan_level(pwm_gpio_to_slice_num(_vref_pin), pwm_gpio_to_channel(_vref_pin), (uint16_t)(wrap * duty_cycle / 100));
    pwm_set_enabled(pwm_gpio_to_slice_num(_vref_pin), true);
    return wrap;
}
