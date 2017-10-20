#include <ros/ros.h>
#include <client/Direction.h>

#include <SFML/Graphics.hpp>

#include <vector>
#include <memory>
#include <fstream>
#include <string>

class Drawable
{
  public:
    virtual void draw(sf::RenderWindow &) = 0;
};

class Map : public Drawable
{
    int width;
    int height;
    std::tuple<int, int> spawn;
    std::vector<std::tuple<int, int>> wall;
    sf::Texture texture;

  public:
    Map()
    {
        std::ifstream in("res/map.txt");
        in >> width >> height;

        std::string row;
        std::getline(in, row);
        for (int j = 0; j < height; ++j)
        {
            std::getline(in, row);
            for (int i = 0; i < width; ++i)
            {
                if (row[i] == '#')
                {
                    wall.push_back(std::make_tuple(i, j));
                }
                if (row[i] == '@')
                {
                    spawn = std::make_tuple(i, j);
                }
            }
        }

        texture.loadFromFile("res/wall.png");
    }

    void draw(sf::RenderWindow &window) override
    {
        for (auto block : wall)
        {
            sf::RectangleShape shape(sf::Vector2f(16, 16));
            shape.setPosition(16 * std::get<0>(block), 16 * std::get<1>(block));
            shape.setTexture(&texture);
            window.draw(shape);
        }
    }

    bool isFreePos(const std::tuple<int, int> &pos)
    {
        for (auto b : wall)
        {
            if (b == pos)
            {
                return false;
            }
        }

        return true;
    }

    std::tuple<int, int> getSpawnPos()
    {
        return spawn;
    }
};

class Hero : public Drawable
{
    int x;
    int y;
    sf::Texture texture;

  public:
    Hero(const std::tuple<int, int> &pos)
    {
        x = std::get<0>(pos);
        y = std::get<1>(pos);
        texture.loadFromFile("res/hero.png");
    }

    void place(std::tuple<int, int> &pos)
    {
        x = std::get<0>(pos);
        y = std::get<1>(pos);
    }

    void draw(sf::RenderWindow &window) override
    {
        sf::RectangleShape shape(sf::Vector2f(16, 16));
        shape.setPosition(16 * x, 16 * y);
        shape.setTexture(&texture);
        window.draw(shape);
    }

    std::tuple<int, int> getPos() { return std::make_tuple(x, y); }
};

class Game : public Drawable
{
    Map map;
    std::unique_ptr<Hero> hero;

  public:
    Game()
    {
        hero.reset(new Hero(map.getSpawnPos()));
    }

    void on_control(const client::Direction &msg)
    {
        std::tuple<int, int> pos = hero->getPos();
        pos = std::make_tuple(std::get<0>(pos) + msg.dx, std::get<1>(pos) + msg.dy);

        if (map.isFreePos(pos))
        {
            hero->place(pos);
        }
    }

    void draw(sf::RenderWindow &window) override
    {
        map.draw(window);
        hero->draw(window);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server");
    ros::Time::init();
    ros::Rate rate(60);
    ros::NodeHandle node;

    sf::RenderWindow window(sf::VideoMode(800, 600), "roguelike", sf::Style::Titlebar | sf::Style::Close);
    Game game;

    ros::Subscriber sub = node.subscribe("/roguelike/control", 10, &Game::on_control, &game);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
        }

        window.clear(sf::Color::Red);
        game.draw(window);
        window.display();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
