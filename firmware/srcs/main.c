#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    while (true)
        printf("Pigeki hello!\n");

    return 0;
}
