#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "externalref/bme280.h"
#include "externalref/tm1637.h"
#include "stdlib.h"
#include "math.h"

#define AFS_SEL_2g 0x00;
#define AFS_SEL_4g 0x08;
#define AFS_SEL_8g 0x10;
#define AFS_SEL_16g 0x18;

#define FS_SEL_250 0x00;  // 250°/s
#define FS_SEL_500 0x08;  // 500°/s
#define FS_SEL_1000 0x10; // 1000°/s
#define FS_SEL_2000 0x18; // 2000°/s
#define CLK_PIN 21
#define DIO_PIN 20
#define I2C1_SDA_PIN 6
#define I2C1_SCL_PIN 7

#define LED_TEMP_PIN 27
#define LED_PRES_PIN 26
#define LED_HUM_PIN 22
#define LED_RELO_PIN 19
#define LED_ESCR_PIN 18

static uint8_t addr_mpu6050 = 0x68;
static uint8_t addr_tm1637 = 0x40;
static uint8_t addr_ds3231 = 0x68;

#ifdef i2c_default
static void mpu6050_reset()
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_default, addr_mpu6050, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00; // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(i2c_default, addr_mpu6050, buf, 2, false);
    sleep_ms(10); // Allow stabilization after waking up

    // Set the gyroscope configuration full scale range to +-500°/s
    buf[0] = 0x1B;       // GYRO_CONFIG
    buf[1] = FS_SEL_500; // FS_SEL 500°/s
    i2c_write_blocking(i2c_default, addr_mpu6050, buf, 2, false);
    sleep_ms(10); // Allow stabilization

    // Set the accelerometer configuration full scale range to +-4g
    buf[0] = 0x1C;       // ACCEL_CONFIG
    buf[1] = AFS_SEL_4g; // AFS_SEL 4g
    i2c_write_blocking(i2c_default, addr_mpu6050, buf, 2, false);
    sleep_ms(10); // Allow stabilization
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr_mpu6050, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr_mpu6050, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr_mpu6050, &val, 1, true);
    i2c_read_blocking(i2c_default, addr_mpu6050, buffer, 6, false); // False - finished with bus

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr_mpu6050, &val, 1, true);
    i2c_read_blocking(i2c_default, addr_mpu6050, buffer, 2, false); // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

// FUNCIÓN DE LECTURA: Firma exacta requerida por struct bme280_dev
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if (reg_data == NULL || len == 0 || intf_ptr == NULL)
        return BME280_E_COMM_FAIL;
    uint8_t dev_addr = *(uint8_t *)intf_ptr;

    if (i2c_write_blocking(i2c_default, dev_addr, &reg_addr, 1, true) < 0)
    {
        return BME280_E_COMM_FAIL;
    }
    if (i2c_read_blocking(i2c_default, dev_addr, reg_data, len, false) < 0)
    {
        return BME280_E_COMM_FAIL;
    }
    return BME280_OK;
}

// FUNCIÓN DE ESCRITURA: Firma exacta requerida por struct bme280_dev
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if (len > 62 || intf_ptr == NULL)
        return BME280_E_COMM_FAIL;

    uint8_t bme_buffer[64];
    uint8_t dev_addr = *(uint8_t *)intf_ptr;

    bme_buffer[0] = reg_addr;
    if (reg_data != NULL && len > 0)
    {
        for (uint32_t i = 0; i < len; i++)
        {
            bme_buffer[i + 1] = reg_data[i];
        }
    }
    if (i2c_write_blocking(i2c_default, dev_addr, bme_buffer, len + 1, false) < 0)
    {
        return BME280_E_COMM_FAIL;
    }
    return BME280_OK;
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
    sleep_us(period);
}

static void ds3231_reset()
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c1, addr_ds3231, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00; // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(i2c1, addr_ds3231, buf, 2, false);
    sleep_ms(10); // Allow stabilization after waking up

    // Set the gyroscope configuration full scale range to +-500°/s
    buf[0] = 0x1B;       // GYRO_CONFIG
    buf[1] = FS_SEL_500; // FS_SEL 500°/s
    i2c_write_blocking(i2c1, addr_ds3231, buf, 2, false);
    sleep_ms(10); // Allow stabilization

    // Set the accelerometer configuration full scale range to +-4g
    buf[0] = 0x1C;       // ACCEL_CONFIG
    buf[1] = AFS_SEL_4g; // AFS_SEL 4g
    i2c_write_blocking(i2c1, addr_ds3231, buf, 2, false);
    sleep_ms(10); // Allow stabilization
}

static uint8_t bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

static uint8_t dec_to_bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

static void ds3231_set_time(uint8_t sec, uint8_t min, uint8_t hour,
                            uint8_t day_of_week, uint8_t day, uint8_t month, uint8_t year)
{
    uint8_t buf[8];
    buf[0] = 0x00;
    buf[1] = dec_to_bcd(sec);
    buf[2] = dec_to_bcd(min);
    buf[3] = dec_to_bcd(hour);
    buf[4] = dec_to_bcd(day_of_week);
    buf[5] = dec_to_bcd(day);
    buf[6] = dec_to_bcd(month);
    buf[7] = dec_to_bcd(year);

    i2c_write_blocking(i2c1, addr_ds3231, buf, 8, false);
}

static void ds3231_read_time(uint8_t *sec, uint8_t *min, uint8_t *hour,
                             uint8_t *day, uint8_t *month, uint8_t *year)
{
    uint8_t reg = 0x00;
    uint8_t data[7];
    i2c_write_blocking(i2c1, addr_ds3231, &reg, 1, true);
    i2c_read_blocking(i2c1, addr_ds3231, data, 7, false);

    *sec = bcd_to_dec(data[0] & 0x7F);
    *min = bcd_to_dec(data[1] & 0x7F);
    *hour = bcd_to_dec(data[2] & 0x3F);
    *day = bcd_to_dec(data[4] & 0x3F);
    *month = bcd_to_dec(data[5] & 0x1F);
    *year = bcd_to_dec(data[6]);
}

static void revisar_sincronizacion_usb(void)
{
    static char rx_buf[32];
    static uint8_t idx = 0;
    int c;

    // Lee todos los caracteres disponibles en el buffer USB sin bloquear el bucle
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT)
    {
        if (c == '\n' || c == '\r')
        {
            if (idx > 0)
            {
                rx_buf[idx] = '\0';
                int Y, M, D, h, m, s;

                // Si llega el formato "SYNC:2026-11-25 20:46:30"
                if (sscanf(rx_buf, "SYNC:%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &m, &s) == 6)
                {
                    ds3231_set_time((uint8_t)s, (uint8_t)m, (uint8_t)h, 1,
                                    (uint8_t)D, (uint8_t)M, (uint8_t)(Y - 2000));
                    printf("DS3231 sincronizado: %04d/%02d/%02d %02d:%02d:%02d\n", Y, M, D, h, m, s);
                }
                idx = 0; // Reiniciar buffer
            }
        }
        else if (idx < sizeof(rx_buf) - 1)
        {
            rx_buf[idx++] = (char)c;
        }
    }
}

int8_t init_led_indi(void)
{
    gpio_init(LED_TEMP_PIN);
    gpio_set_dir(LED_TEMP_PIN, GPIO_OUT);
    gpio_init(LED_PRES_PIN);
    gpio_set_dir(LED_PRES_PIN, GPIO_OUT);
    gpio_init(LED_HUM_PIN);
    gpio_set_dir(LED_HUM_PIN, GPIO_OUT);
    gpio_init(LED_RELO_PIN);
    gpio_set_dir(LED_RELO_PIN, GPIO_OUT);
    gpio_init(LED_ESCR_PIN);
    gpio_set_dir(LED_ESCR_PIN, GPIO_OUT);
}

void clear_led_indi(void)
{
    gpio_put(LED_TEMP_PIN, 0);
    gpio_put(LED_PRES_PIN, 0);
    gpio_put(LED_HUM_PIN, 0);
    gpio_put(LED_RELO_PIN, 0);
    gpio_put(LED_ESCR_PIN, 0);
}
#endif

int main()
{
    stdio_init_all();
    uint32_t status = save_and_disable_interrupts();
    uint64_t f_sec_init = time_us_64();
    restore_interrupts(status);

    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);

    init_led_indi();

    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    const float conversion_factor = 3.3f / (1 << 12);
    const float conversion_acc = (float)4 / (1 << 15);
    // const float conversion_acc_ms2 = ((float)4 / (1 << 15)) * 9.81;
    const float conversion_gyro = (float)500 / (1 << 15);
    static uint8_t dev_addr = BME280_I2C_ADDR_PRIM;
    int n, opt;
    n = opt = 0;
    int8_t rslt = 0;
    uint8_t ss, mm, hh, DD, MM, YYYY;
    uint32_t ffffff;
    uint16_t raw_adc_value = 0;
    int16_t acceleration[3], gyro[3], temp_chip = 0;

    char digits[6];

    struct bme280_data comp_data;
    struct bme280_dev bme;
    bme.delay_us = user_delay_us;
    bme.read = user_i2c_read;
    bme.write = user_i2c_write;
    bme.intf = BME280_I2C_INTF;
    bme.intf_ptr = &dev_addr;

    struct bme280_settings settings;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_16X;
    settings.osr_t = BME280_OVERSAMPLING_2X;
    settings.filter = BME280_FILTER_COEFF_16;
    settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    // Time used to normalize LM35 Vout on Start-Up Response ~40ms (Ref. https://www.ti.com/mx/lit/gpn/lm35)
    sleep_ms(50);

    mpu6050_reset();

    bme280_init(&bme);

    bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme);
    bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme);

    tm1637_init(CLK_PIN, DIO_PIN, 4);

    tm1637_test_segments();

    tm1637_set_brightness(4);

    while (1)
    {
        // ****** DESCOMENTAR SI SE REQUIERE ACTUALIZAR LA HORA Y FECHA DEL RTC ds3231
        // revisar_sincronizacion_usb();

        ds3231_read_time(&ss, &mm, &hh, &DD, &MM, &YYYY);
        ffffff = (uint32_t)(time_us_64() - f_sec_init);

        printf("%04u/%02u/%02u %02u:%02u:%02u.%06lu", 2000 + YYYY, MM, DD, hh, mm, ss, ffffff);

        mpu6050_read_raw(acceleration, gyro, &temp_chip);

        // Acc in g X Y Z
        printf("\t%f\t%f\t%f", acceleration[0] * conversion_acc, acceleration[1] * conversion_acc, acceleration[2] * conversion_acc);
        // Acc in m/s^2 X Y Z
        // printf("\t%f\t%f\t%f", acceleration[0] * conversion_acc_ms2, acceleration[1] * conversion_acc_ms2, acceleration[2] * conversion_acc_ms2);
        // Gyro in °/s X Y Z
        printf("\t%f\t%f\t%f", gyro[0] * conversion_gyro, gyro[1] * conversion_gyro, gyro[2] * conversion_gyro);
        // Note this is chip temperature in °C.
        printf("\t%.2f", (temp_chip / 340.0) + 36.53);

        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);
        if (rslt == BME280_OK)
        {
            printf("\t%.2f\t%.2f\t%.2f", comp_data.temperature, comp_data.pressure / 100.0, comp_data.humidity);
        }
        else
        {
            printf("\tNaN\tNaN\tNaN");
        }
        printf("\n");

        opt = n / 100;
        switch (opt)
        {
        case 0:
            int temperature = (int)comp_data.temperature;
            int decimals = (int)(fabsf(comp_data.temperature - temperature) * 100);

            sprintf(digits, "%02d:%02d", temperature, decimals);
            tm1637_set_colon(true);
            tm1637_show_text(digits, true);
            gpio_put(LED_TEMP_PIN, 1);
            break;
        case 1:
            int presion = (int)floor(comp_data.pressure / 100);

            sprintf(digits, "%04d", presion);
            tm1637_set_colon(false);
            tm1637_show_text(digits, true);
            gpio_put(LED_PRES_PIN, 1);
            break;
        case 2:
            int humedad = (int)comp_data.humidity;
            int hum_decimals = (int)(fabsf(comp_data.humidity - humedad) * 100);

            sprintf(digits, "%02d:%02d", humedad, hum_decimals);
            tm1637_set_colon(true);
            tm1637_show_text(digits, true);
            gpio_put(LED_HUM_PIN, 1);
            break;
        case 3:
            sprintf(digits, "%02u:%02u", hh, mm);
            tm1637_set_colon(true);
            tm1637_show_text(digits, true);
            gpio_put(LED_RELO_PIN, 1);
            break;
        case 4:
            n = 0;
            break;

        default:
            break;
        }

        n++;
        sleep_ms(100);
        clear_led_indi();
    }
}
