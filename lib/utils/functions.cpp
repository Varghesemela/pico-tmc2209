#include "functions.h"


uint32_t pwm_set_freq_duty(uint pwm_pin_num, uint32_t frequency, int duty_cycle){
    uint32_t clock = clock_get_hz(clk_sys);
    frequency = 2*frequency;
    uint32_t divider16 = (clock / frequency) / 4096 +
                            (clock % (frequency * 4096) != 0);
    if (divider16 / 16 == 0) {
        divider16 = 16;
    }

    uint32_t wrap = (((clock * 16) / divider16) / frequency) - 1;
    pwm_set_clkdiv_int_frac(pwm_gpio_to_slice_num(pwm_pin_num), divider16 / 16,
                                        divider16 & 0xF);
    pwm_set_phase_correct(pwm_gpio_to_slice_num(pwm_pin_num), true);
    pwm_set_wrap(pwm_gpio_to_slice_num(pwm_pin_num), wrap);
    pwm_set_chan_level(pwm_gpio_to_slice_num(pwm_pin_num), pwm_gpio_to_channel(pwm_pin_num), wrap * duty_cycle / 100);
    return wrap;
}


float parse_numeric_value(){
    int buffer = 0, decimal_power = 0;
    float var = 0;
    bool neg_data = false, first_byte = true, decimal_data = false;
    while (true)
    {   
        buffer = getchar_timeout_us(0);
        if(first_byte && buffer == '-'){
            neg_data = true;
            first_byte = false;
        }
        if(buffer>='0' && buffer<='9'){    
            var = var*10 + (float)(buffer - 48);
            if (decimal_data){
                decimal_power++;
            }
        }
        else if(buffer == '.'){
            decimal_data = true;
        }
        else if(buffer == '\n' || buffer == ' ' || buffer == ','){
            break;
        }    
        else if (buffer <= 0 || buffer == '-'){
            return PICO_ERROR_TIMEOUT;

        }
    }
    if (neg_data) var = -var;
    if(decimal_data)    var = var/(float)(pow(10, decimal_power));
    return var;
}

void printCommandlist(){
    printf("bXX: LED brightness XX\n");
    printf("sXX : servo angle XX\n");
    printf("u2: Board type\n");
    printf("hz: z encoder home \n");
    printf("dzXX: sets encoder postion of z as XX\n");
    printf("9: Receive current I2C data\n");
    printf("p: Prints encoder value\n");
    printf("x: I2C begin\n");
}
// Function to check
bool is_only_alphabets(string& str){
    return all_of(str.begin(), str.end(), ::isalpha);
}