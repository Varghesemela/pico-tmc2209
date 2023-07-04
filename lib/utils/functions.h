#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <string>
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>
#include "project_config.h"

using namespace std;

float parse_numeric_value();
uint32_t pwm_set_freq_duty(uint pwm_pin_num, uint32_t frequency, int duty_cycle);
void printCommandlist();
bool is_only_alphabets(string& str);


#endif