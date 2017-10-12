#include "lab1/ros_utils.h"
#include "kravchenko_lab1/Move.h"
#include "lab1/move.h"

#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>

RosServer *server;

struct Field {
    char **field;
    int size_y;
    int size_x;
    int hero_pos_x;
    int hero_pos_y;
};

struct Field field = {0};

static void draw_field();

static void init_field()
{
    const char *resources_path = "../../resources/field.txt";
    char field_file_path[strlen(__FILE__) + strlen(resources_path) + 1];
    memset(field_file_path, 0, sizeof(field_file_path));

    int i = 0;
    while (__FILE__[sizeof(__FILE__) - i - 1] != '/')
        i++;

    snprintf(field_file_path, sizeof(field_file_path), "%.*s/%s", (int)strlen(__FILE__) - i, __FILE__, resources_path);

    std::ifstream infile(field_file_path);

    infile >> field.size_y;
    infile >> field.size_x;

    field.field = (char **)malloc(field.size_y * sizeof(char *));

    char newline;
    for (i = 0; i < field.size_y; ++i) {
        field.field[i] = (char *)malloc(field.size_x * sizeof(char));
        infile.get(newline);
        for (int j = 0; j < field.size_x; ++j) {
            infile.get(field.field[i][j]);
            if (field.field[i][j] == '*') {
                field.hero_pos_x = j;
                field.hero_pos_y = i;
            }
        }
    }

    infile.close();

    draw_field();
}

static void clear_field()
{
// WARNING!!!
// This is not a portable solution.
// Tested only under Ubuntu with gnome terminal

    char terminal_clearline [4];
    char terminal_moveup [4];

    sprintf(terminal_clearline, "%c[2K", 0x1B);
    sprintf(terminal_moveup, "%c[1A", 0x1B);

    for (int i = 0; i < field.size_y; i++){
       printf("%s", terminal_moveup);
       printf("%s", terminal_clearline);
    }
    fflush(stdout);
}

static void draw_field()
{
    for (int i = 0; i < field.size_y; ++i) {
        for (int j = 0; j < field.size_x; ++j)
            printf("%c", field.field[i][j]);
        printf("\n");
    }
    fflush(stdout);
}

static bool can_move(struct pos_change pos)
{
    if (field.hero_pos_x + pos.dx < 0
       || field.hero_pos_x + pos.dx >= field.size_x
       || field.hero_pos_y + pos.dy < 0
       || field.hero_pos_y + pos.dy >= field.size_y
       || field.field[field.hero_pos_y + pos.dy][field.hero_pos_x + pos.dx] != '.')
        return false;
    return true;
}

static void move_hero(const kravchenko_lab1::Move& move)
{
    struct pos_change pos = get_move_pos_change(move.move_id);
    if (can_move(pos)) {
        field.field[field.hero_pos_y][field.hero_pos_x] = '.';
        field.hero_pos_x += pos.dx;
        field.hero_pos_y += pos.dy;
        field.field[field.hero_pos_y][field.hero_pos_x] = '*';
    }
}

static void move_handler(const kravchenko_lab1::Move& move)
{
    static int first_time = 1;

    clear_field();

    move_hero(move);
    draw_field();

    first_time = 0;
}

static void init_ros(int argc, char *argv[])
{
    server = new RosServer();
    server->init_ros(argc, argv, (void *)move_handler);
}

int main(int argc, char *argv[])
{
    init_field();
    init_ros(argc, argv);
    server->spin();
}
