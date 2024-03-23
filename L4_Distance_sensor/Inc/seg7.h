/*
 * seg7.h
 *
 *  Created on: Feb 19, 2024
 *      Author: Patryk
 */

#pragma once
#include <stdint.h>

// Pokaż cyfrę na wyświetlaczu
// value - cyfra do wyświetlenia
void seg7_show_digit(uint32_t value);

// Pokaż liczbę na wyświetlaczu
// value - liczba do wyświetlenia
void seg7_show(uint32_t value);

// Funkcja pomocnicza
// Zmiana aktywnego wyświetlacza
void seg7_update(void);
