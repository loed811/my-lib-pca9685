#include "pca9685_i2c_hal.h"
#include "main.h" // Подключаем main.h, чтобы получить доступ к определениям STM32 HAL

/*
 * В main.c (сгенерированном CubeIDE) уже должна быть глобальная переменная:
 * I2C_HandleTypeDef hi2c1;
 * Мы объявляем ее здесь как 'extern', чтобы получить к ней доступ.
 */
extern I2C_HandleTypeDef hi2c1;

// Определяем таймаут для I2C операций
#define I2C_TIMEOUT 100 // 100 мс

/**
 * @brief Инициализация I2C.
 * В нашем случае CubeIDE уже генерирует MX_I2C1_Init(),
 * которая вызывается в main(). Поэтому здесь можно ничего не делать.
 */
int16_t pca9685_i2c_hal_init()
{
    // Проверяем, что I2C1 был инициализирован
    if (hi2c1.Instance == NULL)
    {
        return PCA9685_ERR;
    }

    // HAL_I2C_Init(&hi2c1) уже вызвана в main()
    return PCA9685_OK;
}

/**
 * @brief Чтение данных по I2C для PCA9685.
 * Сначала отправляем адрес регистра (reg), который хотим прочитать,
 * затем читаем 'count' байт данных.
 */
int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count)
{
    HAL_StatusTypeDef status;

    // 1. Отправляем адрес регистра, с которого хотим начать чтение
    // Адрес STM32 HAL ожидает 7-битный адрес, сдвинутый на 1 бит влево
    status = HAL_I2C_Master_Transmit(&hi2c1, (address << 1), reg, 1, I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        return PCA9685_ERR;
    }

    // 2. Читаем 'count' байт данных с устройства
    status = HAL_I2C_Master_Receive(&hi2c1, (address << 1), data, count, I2C_TIMEOUT);

    if (status == HAL_OK)
    {
        return PCA9685_OK;
    }
    else
    {
        return PCA9685_ERR;
    }
}

/**
 * @brief Запись данных по I2C для PCA9685.
 * Библиотека pca9685_i2c.c уже формирует буфер 'data'
 * в формате [АДРЕС_РЕГИСТРА, ДАННЫЕ1, ДАННЫЕ2, ...].
 * Поэтому нам просто нужно отправить весь буфер 'data' размером 'count'.
 */
int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count)
{
    // Адрес STM32 HAL ожидает 7-битный адрес, сдвинутый на 1 бит влево
    if (HAL_I2C_Master_Transmit(&hi2c1, (address << 1), data, count, I2C_TIMEOUT) == HAL_OK)
    {
        return PCA9685_OK;
    }
    else
    {
        return PCA9685_ERR;
    }
}

/**
 * @brief Реализация задержки в миллисекундах.
 * Используем стандартную функцию задержки STM32 HAL.
 */
void pca9685_i2c_hal_ms_delay(uint32_t ms) {
    HAL_Delay(ms);
}
