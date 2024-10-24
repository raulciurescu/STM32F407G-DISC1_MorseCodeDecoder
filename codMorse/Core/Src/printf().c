#include "stm32f4xx_hal.h"  // Include the header file for the STM32 HAL (Hardware Abstraction Layer) library, specific to the STM32F4 series.

extern UART_HandleTypeDef huart3;  // Declare the UART handle for UART3 as external, meaning it’s defined elsewhere in the project.

int _write(int file, char *data, int len)  // Define the _write function, which is typically used to redirect printf() output to a custom destination (e.g., UART).
{
    HAL_UART_Transmit(&huart3, (uint8_t *) data, len, HAL_MAX_DELAY);
    // Transmit the data over UART3. The `data` pointer is cast to `uint8_t*` because HAL functions expect this type.
    // The `len` parameter specifies the number of bytes to send, and `HAL_MAX_DELAY` ensures the function waits indefinitely for the transmission to complete.

    return len;  // Return the length of the data sent, which informs the calling function how many bytes were successfully transmitted.
}
