#include "asf.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

// Config LEDs OLED1
#define LED1_PIO				      PIOA
#define LED1_PIO_ID				    ID_PIOA
#define LED1_PIO_IDX			    0
#define LED1_PIO_IDX_MASK		  (1u << LED1_PIO_IDX)

#define LED2_PIO				      PIOC
#define LED2_PIO_ID				    ID_PIOC
#define LED2_PIO_IDX			    30
#define LED2_PIO_IDX_MASK		  (1u << LED2_PIO_IDX)

#define LED3_PIO				      PIOB
#define LED3_PIO_ID				    ID_PIOB
#define LED3_PIO_IDX			    2
#define LED3_PIO_IDX_MASK		  (1u << LED3_PIO_IDX)

// Config buttons OLED1
#define INCREMENTA_MIN_PIO			  PIOD
#define INCREMENTA_MIN_PIO_ID        ID_PIOD
#define INCREMENTA_MIN_PIO_IDX		  28
#define INCREMENTA_MIN_PIO_IDX_MASK  (1u << INCREMENTA_MIN_PIO_IDX)

#define INCREMENTA_SEC_PIO PIOC
#define INCREMENTA_SEC_PIO_ID ID_PIOC
#define INCREMENTA_SEC_PIO_IDX 31
#define INCREMENTA_SEC_PIO_IDX_MASK (1u << INCREMENTA_SEC_PIO_IDX) // esse já está pronto.

#define START_STOP_PIO			  PIOA
#define START_STOP_PIO_ID           ID_PIOA
#define START_STOP_PIO_IDX		  19
#define START_STOP_PIO_IDX_MASK     (1u << START_STOP_PIO_IDX)
/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

volatile char flag_rtc_alarm = 0;
volatile char flag_rtc_sec = 0;
volatile char flag_incrementa_min = 0;
volatile char flag_incrementa_sec = 0;
volatile char flag_start_stop = 0;
volatile char flag_tc_sec = 0;



/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void init(void);
void pin_toggle(Pio *pio, uint32_t mask);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void display_date(uint32_t, uint32_t, uint32_t);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

void incrementa_min_callback(void){
	if (!pio_get(INCREMENTA_MIN_PIO, PIO_INPUT, INCREMENTA_MIN_PIO_IDX_MASK)) {
		// PINO == 0 --> Borda de descida
		flag_incrementa_min = 1;
	} else {
		// PINO == 1 --> Borda de subida
		flag_incrementa_min = 0;
	}
}

void incrementa_sec_callback(void){
	if (!pio_get(INCREMENTA_SEC_PIO, PIO_INPUT, INCREMENTA_SEC_PIO_IDX_MASK)) {
		// PINO == 0 --> Borda de descida
		flag_incrementa_sec = 1;
	} else {
		// PINO == 1 --> Borda de subida
		flag_incrementa_sec = 0;
	}
}

void start_stop_callback(void){
	flag_start_stop = 1;
}

void TC1_Handler(void) {
	volatile uint32_t status = tc_get_status(TC0, 1);
	flag_tc_sec = 1;
}


void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		flag_rtc_sec = 1;
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		flag_rtc_alarm = 1;
	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

void configure_output(Pio *p_pio, const pio_type_t ul_type, const uint32_t ul_mask, const uint32_t ul_attribute, uint32_t ul_id ){
	pmc_enable_periph_clk(ul_id);
	pio_configure(p_pio, ul_type, ul_mask, ul_attribute);
}

void configure_input(Pio *p_pio, const pio_type_t ul_type, const uint32_t ul_mask, const uint32_t ul_attribute, uint32_t ul_id){
	pmc_enable_periph_clk(ul_id);
	pio_configure(p_pio, ul_type, ul_mask, ul_attribute);
	pio_set_debounce_filter(p_pio, ul_mask, 60);
}

void configure_interruption(Pio *p_pio, uint32_t ul_id, uint32_t ul_mask, uint32_t ul_attr, void (*p_handler) (uint32_t, uint32_t)){
	pio_handler_set(p_pio, ul_id, ul_mask, ul_attr, p_handler);
	pio_enable_interrupt(p_pio, ul_mask);
	pio_get_interrupt_status(p_pio);
	NVIC_EnableIRQ(ul_id);
	NVIC_SetPriority(ul_id, 4);
}


void init(void) {
	// Initialize the board clock
	sysclk_init();

	// Deactivate WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// LEDs OLED1
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 1, 0, 1);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 1, 0, 1);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 1, 0, 1);
	
	
	
	configure_input(INCREMENTA_MIN_PIO, PIO_INPUT, INCREMENTA_MIN_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE, INCREMENTA_MIN_PIO_ID);
	configure_input(INCREMENTA_SEC_PIO, PIO_INPUT, INCREMENTA_SEC_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE, INCREMENTA_SEC_PIO_ID);
	configure_input(START_STOP_PIO, PIO_INPUT, START_STOP_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE, START_STOP_PIO_ID);
	
	configure_interruption(INCREMENTA_MIN_PIO, INCREMENTA_MIN_PIO_ID, INCREMENTA_MIN_PIO_IDX_MASK, PIO_IT_EDGE, incrementa_min_callback);
	configure_interruption(INCREMENTA_SEC_PIO, INCREMENTA_SEC_PIO_ID, INCREMENTA_SEC_PIO_IDX_MASK, PIO_IT_EDGE, incrementa_sec_callback);
	configure_interruption(START_STOP_PIO, START_STOP_PIO_ID, START_STOP_PIO_IDX_MASK, PIO_IT_FALL_EDGE, start_stop_callback);
}

/**
* @Brief Inverte o valor do pino 0->1/ 1->0
*/
void pin_toggle(Pio *pio, uint32_t mask) {
  if(pio_get_output_data_status(pio, mask))
		pio_clear(pio, mask);
  else
		pio_set(pio,mask);
}


void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
  	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}


void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}


void display_date(uint32_t hour, uint32_t min, uint32_t sec){
	char date_str[128];
	sprintf(date_str, "%02d:%02d:%02d", hour, min, sec);
	gfx_mono_draw_string(date_str, 0, 0, &sysfont);
}

void display_timer(uint32_t min, uint32_t sec){
	char timer_str[128];
	sprintf(timer_str, "%02d:%02d", min, sec);
	gfx_mono_draw_string(timer_str, 20, 18, &sysfont);
}

void start_timer(uint32_t min, uint32_t sec){
	TC_init(TC0, ID_TC1, 1, 1);
	tc_start(TC0, 1);
	
	uint32_t current_min = min;
	uint32_t current_sec = sec;
	
	int counter = 0;
	while(min*60 + sec > counter){
		if(flag_tc_sec){
			if(current_sec == 0){
				current_sec = 59;
				current_min-=1;
			} else {
				current_sec-=1;
			}
			display_timer(current_min, current_sec);
			flag_tc_sec = 0;
			counter++;
		}
	}
	pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
	tc_stop(TC0, 1);
}

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	board_init();
	delay_init();

	init();

	gfx_mono_ssd1306_init();
	
	calendar rtc_initial = {2022, 4, 6, 1, 16, 0, 0};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_SR_SEC|RTC_SR_ALARM);

	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	
	uint32_t timer_min = 0;
	uint32_t timer_sec = 0;
	
	display_timer(timer_min, timer_sec);
	
	float acel = 1;

	while (1) {
		if (flag_rtc_sec) {
			flag_rtc_sec = 0;
			rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			display_date(current_hour, current_min, current_sec);
		} else if (flag_incrementa_min){
			if(timer_min < 59){
				timer_min++;
				if(acel < 10){
					acel +=0.1;
				}
			}	
			display_timer(timer_min,timer_sec);
			delay_ms(500/acel);
		} else if (flag_incrementa_sec){
			if(timer_sec < 59){
				timer_sec++;
				if(acel < 10){
					acel +=0.1;
				}
			}
			display_timer(timer_min,timer_sec);
			delay_ms(500/acel);
		} else if (flag_start_stop){
			pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
			start_timer(timer_min, timer_sec);
			flag_start_stop = 0;
			timer_min = 0;
			timer_sec = 0;
			display_timer(timer_min, timer_sec);
			pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
		} else if (flag_incrementa_min || flag_incrementa_sec){
			acel = 1; 
		}else {
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);	
		}
	}
}