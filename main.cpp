#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#include <FixedPoint/FixedPoint.hpp>
#include "Smoother.hpp"
#include "PID.hpp"
#include "SigmaDelta.hpp"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>

typedef FixedPoint<4, 27> Number;
typedef FixedPoint<12, 19> BigNumber;
typedef FixedPoint<12, 51> HugeNumber;

#define PWM_DEBUGGING 1

class State {
public:
    Smoother<Number, 1> i;
    Smoother<Number, 1> q;
    Smoother<Number, 1> phase;
    int32_t phase_count;
    int32_t iter;

    constexpr State()
        : i(0.004, 0.)
        , q(0.004, 1.)
        , phase(0.01, 0)
        , phase_count(0)
        , iter(0)
    {
    }

    void process_measurement(Number vn)
    {
        switch(iter) {
        case 0:
            i << vn;
            break;
        case 1:
            q << vn;
            break;
        case 2:
            i << -vn;
            break;
        case 3:
            q << -vn;
            break;
        }
        iter = (iter+1) & 3;
    }

    Number calculate_phase()
    {
        Number old_phase = *phase;
        Number new_phase = std::atan2(*q, *i);
        Number phase_diff = 0;
        int changed_count = 0;
        if (new_phase < old_phase && std::abs(new_phase-old_phase) > Number(M_PI)) {
            phase_diff = PI2;
            phase_count++;
            changed_count = +1;
        } else if (new_phase > old_phase && std::abs(new_phase-old_phase) > Number(M_PI)) {
            phase_diff = -PI2;
            phase_count--;
            changed_count = -1;
        }
        phase.add_all(-phase_diff);
        phase << new_phase;

        if (changed_count) {
            if (std::abs(*phase) > Number(M_PI)) {
                phase_count -= changed_count;
                phase.add_all(phase_diff);
                phase_diff = 0.;
            }
        }

        return *phase - old_phase + phase_diff;
    }

protected:
    static constexpr Number PI2 = Number(2*M_PI);
};
constexpr Number State::PI2;

static State state = State();

void setup_gpios(void) {
#ifdef PWM_DEBUGGING
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
#endif
}

void setup_clocks(void) {
    rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);
    rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);
    rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);

    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);
    rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

    flash_set_ws(FLASH_ACR_LATENCY_2WS); // 0 with 0-24, 1 with 24-48, 2 with 48-72 MHz

    rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL14);
    rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);
    rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK_DIV2);

    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);

    rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

    uint32_t hse_frequency = 8000000;
    rcc_ahb_frequency = hse_frequency / 2 * 14;
    rcc_apb1_frequency = hse_frequency / 2 * 14 / 2;
    rcc_apb2_frequency = hse_frequency / 2 * 14;
}

void setup_adc(void) {
    // First setup the adc, after powering it off
    adc_power_off(ADC1);

    // Enable clock for the ADC itself and the needed input port, then set the input channel mode
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);

    // No scan mode, single conversion, triggered by timer 3
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_TIM3_TRGO);

    // Gotta go fast
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_7DOT5CYC);

    // Use channel 0
    uint8_t adc_channels[] = { ADC_CHANNEL0 };
    adc_set_regular_sequence(ADC1, sizeof(adc_channels)/sizeof(adc_channels[0]), adc_channels);

    // Power it on again
    adc_power_on(ADC1);

    // Wait a little, then calibrate
    for (volatile unsigned int i = 0; i < 10000; i++) __asm__("nop");
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

    // Enable the end of conversion interrupt
    nvic_enable_irq(NVIC_ADC1_2_IRQ);
    adc_enable_eoc_interrupt(ADC1);

    // Configure timer 3 to run at 10 kHz and have it trigger on overflow
    rcc_periph_clock_enable(RCC_TIM3);
    timer_reset(TIM3);
    timer_set_mode(TIM3,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    timer_update_on_overflow(TIM3);
    timer_set_period(TIM3, rcc_apb1_frequency / 10000 * 2);
    timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);
    timer_enable_counter(TIM3);
}

void setup_pwm(void) {
    // Use timer 1, and enable the AFIO clock.
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_AFIO);

    // Start clock for the needed output pins
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM1_CH1);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM1_CH2);

    // General settings
    TIM1_CR1 &= ~TIM_CR1_CEN;
    TIM1_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_ARPE | TIM_CR1_CMS_EDGE;
    TIM1_PSC = 0;
    TIM1_ARR = 4096;
    TIM1_EGR = TIM_EGR_UG;

#ifdef PWM_DEBUGGING
    // Channel 1
    TIM1_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
    TIM1_CCER = TIM_CCER_CC1E;
    TIM1_CCR1 = 2048;

    // Channel 2
    TIM1_CCMR1 |= TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
    TIM1_CCER |= TIM_CCER_CC2E;
    TIM1_CCR2 = 2048;
#endif

    // Run
    TIM1_CR1 |= TIM_CR1_ARPE;
    TIM1_BDTR |= TIM_BDTR_MOE;
    TIM1_CR1 |= TIM_CR1_CEN;
}

static volatile  int32_t current_measurement = 0;
static volatile uint32_t got_new_measurement = 0;
SigmaDelta<BigNumber, int32_t> sdm;
static constexpr PID<HugeNumber, true, true, true> pidstart(
        /*kp*/       4. * 0.2 / 2,
        /*ki*/       4. * 0.2 / (0.2/2.) / 4,
        /*kd*/       4. * 0.2 * (0.2/3.),
        /*interval*/ 1e-3,
        /*min*/      -16.,
        /*max*/      +16.,
        /*setpoint*/ 0.);
int main() {
    PID<HugeNumber, true, true, true> pid = pidstart;

    setup_gpios();
    setup_clocks();
    setup_adc();
    setup_pwm();

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_TIM1_CH3);

    int iter = 0;
    BigNumber phase = 0;
    gpio_toggle(GPIOA, GPIO_TIM1_CH2);
    while(1) {
        __asm__("wfi");
        if (got_new_measurement) {
            Number v = 3.6/2048;
            v *= int64_t(current_measurement);
            state.process_measurement(v);
            if (++iter == 10) {
                iter = 0;
                state.calculate_phase();
                // Don't bother to calculate huge errors, just use ±pi
                if (state.phase_count < 0) {
                    phase = -M_PI;
                } else if (state.phase_count > 0) {
                    phase = M_PI;
                } else {
                    phase = *state.phase;
                }
                pid.step(phase);
                sdm.setpoint = pid.output();
#ifdef PWM_DEBUGGING
                // Debugging stuff: Show the phase and PID output with PWM until we have something better (USB?)
                if (std::abs(phase) > BigNumber(M_PI/10)) {
                    if (phase.is_negative()) {
                        phase = -M_PI/10;
                    } else {
                        phase = M_PI/10;
                    }
                }
                TIM1_CCR1 = int32_t(BigNumber(phase)*BigNumber(2040./M_PI))+2048;
                TIM1_CCR2 = int32_t(BigNumber(pid.output())*int64_t(2040./64.)) + 2048;
#endif
            }
        }
    }
}


void adc1_2_isr(void) {
#ifdef PWM_DEBUGGING
    gpio_toggle(GPIOA, GPIO_TIM1_CH3);
#endif
    current_measurement = adc_read_regular(ADC1) - (1<<11);
    got_new_measurement = 1;
    sdm.step();
    timer_set_period(TIM3, 5600 + sdm.output());
}
