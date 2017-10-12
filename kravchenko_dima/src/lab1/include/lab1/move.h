#pragma once

#include <stddef.h>

struct pos_change {
    int dx;
    int dy;
};

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

static inline struct pos_change get_move_pos_change(int move)
{
    struct pos_change pos = {0};
    switch (move) {
        case UP:
            pos = {0, -1};
            break;
        case DOWN:
            pos = {0, 1};
            break;
        case RIGHT:
            pos = {1, 0};
            break;
        case LEFT:
            pos = {-1, 0};
            break;
    }
    return pos;
}
