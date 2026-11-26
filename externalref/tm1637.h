
#ifndef TM1637_H
#define TM1637_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * Inicializa el TM1637.
     * brightness: de 0 (mínimo) a 7 (máximo).
     */
    void tm1637_init(uint clk_pin, uint dio_pin, uint8_t brightness);

    /** Ajusta el brillo entre 0 y 7. */
    void tm1637_set_brightness(uint8_t brightness);

    /** Enciende o apaga la pantalla completa. */
    void tm1637_set_display(bool on);

    /** Borra los cuatro dígitos. */
    void tm1637_clear(void);

    /** Controla los dos puntos centrales. */
    void tm1637_set_colon(bool on);

    /**
     * Escribe un dígito hexadecimal: 0-9, A-F.
     * position: 0 es el dígito izquierdo; 3, el derecho.
     * dot: activa el punto decimal individual.
     */
    void tm1637_set_digit(uint8_t position, char digit, bool dot);

    /**
     * Escribe hasta cuatro dígitos hexadecimales.
     * Admite puntos decimales y dos puntos:
     *   "23.45" -> cuatro dígitos, con punto decimal.
     *   "23:45" -> cuatro dígitos, con dos puntos centrales.
     * left_align: true alinea a la izquierda; false a la derecha.
     */
    void tm1637_show_text(const char *text, bool left_align);

    /**
     * Muestra un entero de 0 a 0xFFFF en hexadecimal.
     * leading_zeros controla los ceros iniciales.
     */
    void tm1637_show_hex(uint16_t value, bool leading_zeros);

    void tm1637_test_segments();

#ifdef __cplusplus
}
#endif

#endif
