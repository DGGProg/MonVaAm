
#include "tm1637.h"

#define TM1637_CMD_DATA 0x40
#define TM1637_CMD_ADDRESS 0xC0
#define TM1637_CMD_DISPLAY 0x88

#define TM1637_DIGITS 4
#define TM1637_DP 0x80

static uint pin_clk;
static uint pin_dio;
static uint8_t display_brightness;
static bool display_enabled = true;
static bool central_colon = false;

/*
 * Codificación de segmentos:
 *
 *       A
 *      ---
 *   F |   | B
 *      -G-
 *   E |   | C
 *      ---
 *       D
 *
 * Bit 7: punto decimal (DP).
 */
static const uint8_t digit_segments[16] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71  // F
};

static uint8_t display_buffer[TM1637_DIGITS] = {
    0, 0, 0, 0};

static void tm1637_delay(void)
{
    sleep_us(5);
}

/* CLK y DIO se controlan directamente desde GPIO. */

static void tm1637_start(void)
{
    gpio_set_dir(pin_dio, GPIO_OUT);
    gpio_put(pin_clk, 1);
    gpio_put(pin_dio, 1);
    tm1637_delay();

    gpio_put(pin_dio, 0);
    tm1637_delay();

    gpio_put(pin_clk, 0);
    tm1637_delay();
}

static void tm1637_stop(void)
{
    gpio_set_dir(pin_dio, GPIO_OUT);
    gpio_put(pin_clk, 0);
    gpio_put(pin_dio, 0);
    tm1637_delay();

    gpio_put(pin_clk, 1);
    tm1637_delay();

    gpio_put(pin_dio, 1);
    tm1637_delay();
}

/* Envía un byte, empezando por el bit menos significativo. */
static bool tm1637_write_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        gpio_put(pin_clk, 0);
        gpio_put(pin_dio, data & 0x01);
        tm1637_delay();

        gpio_put(pin_clk, 1);
        tm1637_delay();

        data >>= 1;
    }

    /* Ciclo de ACK del TM1637. */
    gpio_put(pin_clk, 0);
    gpio_set_dir(pin_dio, GPIO_IN);
    tm1637_delay();

    gpio_put(pin_clk, 1);
    tm1637_delay();

    bool ack = (gpio_get(pin_dio) == 0);

    gpio_put(pin_clk, 0);
    gpio_set_dir(pin_dio, GPIO_OUT);
    tm1637_delay();

    return ack;
}

static void tm1637_send_command(uint8_t command)
{
    tm1637_start();
    (void)tm1637_write_byte(command);
    tm1637_stop();
}

static void tm1637_refresh(void)
{
    uint8_t data[TM1637_DIGITS];

    for (int i = 0; i < TM1637_DIGITS; i++)
    {
        data[i] = display_buffer[i];
    }

    /*
     * En muchos módulos el colon está conectado al bit DP
     * del segundo dígito (índice 1).
     */
    if (central_colon)
    {
        data[1] |= TM1637_DP;
    }

    tm1637_send_command(TM1637_CMD_DATA);

    tm1637_start();
    (void)tm1637_write_byte(TM1637_CMD_ADDRESS);

    for (int i = 0; i < TM1637_DIGITS; i++)
    {
        (void)tm1637_write_byte(data[i]);
    }

    tm1637_stop();

    uint8_t command = TM1637_CMD_DISPLAY | (display_brightness & 0x07);

    if (!display_enabled)
    {
        command = 0x80;
    }

    tm1637_send_command(command);
}

static int tm1637_hex_value(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }

    if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }

    if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }

    return -1;
}

void tm1637_init(uint clk_pin, uint dio_pin, uint8_t brightness)
{
    pin_clk = clk_pin;
    pin_dio = dio_pin;

    display_brightness = brightness & 0x07;
    display_enabled = true;
    central_colon = false;

    gpio_init(pin_clk);
    gpio_init(pin_dio);

    gpio_set_dir(pin_clk, GPIO_OUT);
    gpio_set_dir(pin_dio, GPIO_OUT);

    gpio_put(pin_clk, 1);
    gpio_put(pin_dio, 1);

    sleep_ms(10);

    tm1637_clear();
    tm1637_refresh();
}

void tm1637_set_brightness(uint8_t brightness)
{
    display_brightness = brightness & 0x07;
    tm1637_refresh();
}

void tm1637_set_display(bool on)
{
    display_enabled = on;
    tm1637_refresh();
}

void tm1637_clear(void)
{
    for (int i = 0; i < TM1637_DIGITS; i++)
    {
        display_buffer[i] = 0;
    }

    central_colon = false;
    tm1637_refresh();
}

void tm1637_set_colon(bool on)
{
    central_colon = on;
    tm1637_refresh();
}

void tm1637_set_digit(uint8_t position, char digit, bool dot)
{
    if (position >= TM1637_DIGITS)
    {
        return;
    }

    int value = tm1637_hex_value(digit);

    if (value < 0)
    {
        display_buffer[position] = 0;
    }
    else
    {
        display_buffer[position] = digit_segments[value];

        if (dot)
        {
            display_buffer[position] |= TM1637_DP;
        }
    }

    tm1637_refresh();
}

void tm1637_show_text(const char *text, bool left_align)
{
    if (text == NULL)
    {
        return;
    }

    uint8_t segments[TM1637_DIGITS] = {0};
    int count = 0;
    bool colon = false;

    for (int i = 0; text[i] != '\0'; i++)
    {
        char c = text[i];

        if (c == ':')
        {
            colon = true;
            continue;
        }

        if (c == '.')
        {
            if (count > 0)
            {
                segments[count - 1] |= TM1637_DP;
            }
            continue;
        }

        int value = tm1637_hex_value(c);

        if (value < 0)
        {
            continue;
        }

        if (count >= TM1637_DIGITS)
        {
            break;
        }

        segments[count++] = digit_segments[value];
    }

    for (int i = 0; i < TM1637_DIGITS; i++)
    {
        display_buffer[i] = 0;
    }

    int start = left_align ? 0 : TM1637_DIGITS - count;

    for (int i = 0; i < count; i++)
    {
        display_buffer[start + i] = segments[i];
    }

    central_colon = colon;
    tm1637_refresh();
}

void tm1637_show_hex(uint16_t value, bool leading_zeros)
{
    char text[5];

    for (int i = 3; i >= 0; i--)
    {
        uint8_t nibble = (value >> (i * 4)) & 0x0F;

        text[3 - i] = "0123456789ABCDEF"[nibble];
    }

    text[4] = '\0';

    if (leading_zeros)
    {
        tm1637_show_text(text, true);
    }
    else
    {
        int first = 0;

        while (first < 3 && text[first] == '0')
        {
            first++;
        }

        tm1637_show_text(&text[first], false);
    }
}

void tm1637_test_segments(void)
{
        for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            display_buffer[j] = 0;
        }

        central_colon = false;
        display_buffer[i] = 0xFF;

        tm1637_refresh();
        sleep_ms(1500);
    }
}