#include "bsp_nrf24.h"
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
#include <string.h>

/* Global Variables */
RemoteControl_Data_t g_nrf24_rx_data = {0};
NRF24_Status_t g_nrf24_status = {0};
uint8_t g_nrf24_rx_buffer[NRF24_PAYLOAD_SIZE] = {0};
volatile bool g_nrf24_data_ready = false;

/* Global Debug Variables - can be watched in debugger */
volatile uint8_t g_nrf24_debug_config = 0;
volatile uint8_t g_nrf24_debug_status = 0;
volatile uint8_t g_nrf24_debug_channel = 0;
volatile uint8_t g_nrf24_debug_setup = 0;
volatile bool g_nrf24_debug_data_available = false;
volatile uint32_t g_nrf24_debug_data_check_count = 0;
volatile uint32_t g_nrf24_debug_receive_attempts = 0;
volatile uint8_t g_nrf24_debug_fifo_status = 0;
volatile uint8_t g_nrf24_debug_last_status = 0;
volatile bool g_nrf24_debug_buffer_non_zero = false;

/* Private Variables */
static uint8_t rx_address[NRF24_ADDRESS_WIDTH] = {0x45, 0x55, 0x67, 0x10, 0x21};
static uint32_t last_packet_time = 0;

// Non-blocking buzzer state machine
static struct {
    bool active;
    uint8_t beep_count;
    uint8_t current_beep;
    uint32_t next_action_time;
    bool buzzer_on;
} buzzer_state = {false, 0, 0, 0, false};

/**
 * @brief Non-blocking buzzer signal
 * @param beep_count Number of beeps
 */
void BSP_NRF24_BuzzerSignal_NonBlocking(uint8_t beep_count)
{
    buzzer_state.active = true;
    buzzer_state.beep_count = beep_count;
    buzzer_state.current_beep = 0;
    buzzer_state.next_action_time = HAL_GetTick() + 50; // Start after 50ms
    buzzer_state.buzzer_on = false;
}

/**
 * @brief Update non-blocking buzzer (call in main loop)
 */
void BSP_NRF24_BuzzerUpdate(void)
{
    if (!buzzer_state.active) {
        return;
    }
    
    uint32_t current_time = HAL_GetTick();
    if (current_time < buzzer_state.next_action_time) {
        return;
    }
    
    if (!buzzer_state.buzzer_on) {
        // Turn on buzzer
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        buzzer_state.buzzer_on = true;
        buzzer_state.next_action_time = current_time + 100; // Beep for 100ms
    } else {
        // Turn off buzzer
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        buzzer_state.buzzer_on = false;
        buzzer_state.current_beep++;
        
        if (buzzer_state.current_beep >= buzzer_state.beep_count) {
            // All beeps done
            buzzer_state.active = false;
        } else {
            // Schedule next beep
            buzzer_state.next_action_time = current_time + 150; // 150ms pause
        }
    }
}

/**
 * @brief Minimal GPIO initialization (avoid conflicts)
 */
void BSP_NRF24_GPIO_Init_Minimal(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clocks
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // Configure CSN pin (PE15) as output
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    // Configure CE pin (PD10) as output
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // Configure Buzzer pin (PB15) as output
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Initialize pin states
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);  // CSN high
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // CE low
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // Buzzer off
    
    // Note: We skip PA5 IRQ configuration to avoid conflicts
}

/**
 * @brief Non-blocking NRF24 initialization
 * @retval true if initialization successful, false otherwise
 */
bool BSP_NRF24_Init_NonBlocking(void)
{
    // Minimal GPIO initialization to avoid conflicts
    BSP_NRF24_GPIO_Init_Minimal();
    
    // Start non-blocking buzzer signal
    BSP_NRF24_BuzzerSignal_NonBlocking(1);
    
    // Initialize status structure
    memset(&g_nrf24_status, 0, sizeof(NRF24_Status_t));
    memset(&g_nrf24_rx_data, 0, sizeof(RemoteControl_Data_t));
    
    // Initialize NRF24 pins to safe state
    csn_high();  // CSN high (deselect)
    ce_low();    // CE low (standby)
    
    // Non-blocking delay - just mark init time
    static uint32_t init_start_time = 0;
    if (init_start_time == 0) {
        init_start_time = HAL_GetTick();
        return false; // Not ready yet
    }
    
    // Wait 110ms for power-up without blocking
    if ((HAL_GetTick() - init_start_time) < 110) {
        return false; // Still waiting
    }
    
    // Quick SPI test
    uint8_t test_reg = nrf24_r_reg(CONFIG, 1);
    
    // Initialize NRF24 module with COMPLETE configuration to match TX
    nrf24_init();
    
    // Configure as receiver
    nrf24_listen();
    
    // CRITICAL: Set ALL parameters to match transmitter exactly
    nrf24_auto_ack_all(auto_ack);           // Enable auto acknowledgment
    nrf24_en_ack_pld(disable);              // Disable ACK payload
    nrf24_dpl(disable);                     // Disable dynamic payload length
    nrf24_set_crc(en_crc, _1byte);          // Enable 1-byte CRC
    nrf24_tx_pwr(_0dbm);                    // Set TX power to 0dBm
    nrf24_data_rate(_1mbps);                // Set data rate to 1Mbps
    nrf24_set_channel(NRF24_CHANNEL);       // Set channel to 90
    nrf24_set_addr_width(NRF24_ADDRESS_WIDTH); // Set address width to 5 bytes
    
    // Configure payload size for all pipes
    nrf24_pipe_pld_size(0, NRF24_PAYLOAD_SIZE);
    nrf24_set_rx_dpl(0, disable);
    nrf24_set_rx_dpl(1, disable);
    nrf24_set_rx_dpl(2, disable);
    nrf24_set_rx_dpl(3, disable);
    nrf24_set_rx_dpl(4, disable);
    nrf24_set_rx_dpl(5, disable);
    
    // Configure auto retransmission to match TX
    nrf24_auto_retr_delay(NRF24_AUTO_RETRANS_DELAY);
    nrf24_auto_retr_limit(NRF24_AUTO_RETRANS_COUNT);
    
    // Open RX pipe with specified address
    nrf24_open_rx_pipe(0, rx_address);
    
    // Start listening
    ce_high();
    
    // Update status
    g_nrf24_status.is_initialized = true;
    g_nrf24_status.connection_established = false;
    g_nrf24_status.data_receiving = false;
    g_nrf24_status.timeout_error = false;
    
    return true;
}

/**
 * @brief Non-blocking data reception (optimized for main loop)
 * @retval true if new data received, false otherwise
 */
bool BSP_NRF24_ReceiveData_NonBlocking(void)
{
    if (!g_nrf24_status.is_initialized) {
        return false;
    }
    
    // Increment debug counter
    g_nrf24_debug_data_check_count++;
    
    // Read FIFO status for debugging
    g_nrf24_debug_fifo_status = nrf24_r_reg(FIFO_STATUS, 1);
    g_nrf24_debug_last_status = nrf24_r_status();
    
    // Quick check for data availability
    bool data_available = nrf24_data_available();
    g_nrf24_debug_data_available = data_available;
    
    if (!data_available) {
        return false;
    }
    
    // Data is available - increment receive attempts
    g_nrf24_debug_receive_attempts++;
    
    // Clear buffer and receive
    memset(g_nrf24_rx_buffer, 0, NRF24_PAYLOAD_SIZE);
    nrf24_receive(g_nrf24_rx_buffer, NRF24_PAYLOAD_SIZE);
    
    // Check if buffer has non-zero data for debugging
    g_nrf24_debug_buffer_non_zero = false;
    for (int i = 0; i < NRF24_PAYLOAD_SIZE; i++) {
        if (g_nrf24_rx_buffer[i] != 0) {
            g_nrf24_debug_buffer_non_zero = true;
            break;
        }
    }
    
    // Always process data even if it's all zeros (might be valid data)
    g_nrf24_rx_data.adc_ch1 = (g_nrf24_rx_buffer[1] << 8) | g_nrf24_rx_buffer[0];
    g_nrf24_rx_data.adc_ch2 = (g_nrf24_rx_buffer[3] << 8) | g_nrf24_rx_buffer[2];
    g_nrf24_rx_data.adc_ch3 = (g_nrf24_rx_buffer[5] << 8) | g_nrf24_rx_buffer[4];
    g_nrf24_rx_data.adc_ch4 = (g_nrf24_rx_buffer[7] << 8) | g_nrf24_rx_buffer[6];
    g_nrf24_rx_data.aux1_state = g_nrf24_rx_buffer[8];
    g_nrf24_rx_data.aux2_state = g_nrf24_rx_buffer[9];
    
    // Simple validation
    if (g_nrf24_rx_data.adc_ch1 > 4095) g_nrf24_rx_data.adc_ch1 = 2048;
    if (g_nrf24_rx_data.adc_ch2 > 4095) g_nrf24_rx_data.adc_ch2 = 2048;
    if (g_nrf24_rx_data.adc_ch3 > 4095) g_nrf24_rx_data.adc_ch3 = 2048;
    if (g_nrf24_rx_data.adc_ch4 > 4095) g_nrf24_rx_data.adc_ch4 = 2048;
    
    // Update status - FIXED: Set data_receiving flag
    g_nrf24_status.data_receiving = true;
    g_nrf24_status.total_packets++;
    g_nrf24_status.last_receive_time = HAL_GetTick();
    last_packet_time = HAL_GetTick();
    
    // Check if this is first successful communication
    if (!g_nrf24_status.connection_established) {
        g_nrf24_status.connection_established = true;
        BSP_NRF24_BuzzerSignal_NonBlocking(2); // Non-blocking beeps
    }
    
    g_nrf24_status.timeout_error = false;
    
    return true;
}

/**
 * @brief Debug function to check NRF24 status and configuration
 * @retval true if NRF24 seems to be working, false otherwise
 */
bool BSP_NRF24_DebugStatus(void)
{
    if (!g_nrf24_status.is_initialized) {
        return false;
    }
    
    // Read some key registers to verify NRF24 is responding
    g_nrf24_debug_config = nrf24_r_reg(CONFIG, 1);
    g_nrf24_debug_status = nrf24_r_reg(STATUS, 1);
    g_nrf24_debug_channel = nrf24_r_reg(RF_CH, 1);
    g_nrf24_debug_setup = nrf24_r_reg(RF_SETUP, 1);
    
    // Basic sanity checks
    if (g_nrf24_debug_channel == NRF24_CHANNEL && 
        (g_nrf24_debug_config & 0x01) == 1 &&  // PWR_UP bit should be 1
        (g_nrf24_debug_config & 0x02) == 1) {  // PRIM_RX bit should be 1 for receiver
        return true;
    }
    
    return false;
}

/**
 * @brief Lightweight status update (call every 100ms or so)
 */
void BSP_NRF24_UpdateStatus_Lightweight(void)
{
    static uint32_t last_status_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Only update status every 100ms to reduce CPU load
    if ((current_time - last_status_update) < 100) {
        return;
    }
    last_status_update = current_time;
    
    // Update buzzer state machine
    BSP_NRF24_BuzzerUpdate();
    
    // Check for timeout
    if (g_nrf24_status.connection_established && 
        (current_time - last_packet_time) > NRF24_TIMEOUT_MS) {
        g_nrf24_status.timeout_error = true;
        g_nrf24_status.connection_established = false;
        g_nrf24_status.lost_packets++;
        
        // Reset to center values on timeout
        g_nrf24_rx_data.adc_ch1 = 2048;
        g_nrf24_rx_data.adc_ch2 = 2048;
        g_nrf24_rx_data.adc_ch3 = 2048;
        g_nrf24_rx_data.adc_ch4 = 2048;
        g_nrf24_rx_data.aux1_state = 0;
        g_nrf24_rx_data.aux2_state = 0;
    }
} 