#pragma once

#include <stddef.h>

enum move {
    UP = 'A',
    DOWN = 'B',
    RIGHT = 'C',
    LEFT = 'D'
};

static inline const char *get_move_name(int move)
{
    switch (move) {
        case UP:
            return "UP";
        case DOWN:
            return "DOWN";
        case RIGHT:
            return "RIGHT";
        case LEFT:
            return "LEFT";
    }
    return NULL;
}
