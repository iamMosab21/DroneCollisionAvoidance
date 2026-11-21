#include <SFML/Graphics.hpp>
#include "drone_controller.hpp"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>

int main() {
    const int screenWidth = 900;
    const int screenHeight = 900;
    const float droneRadius = 10.0f;
    const float maxSpeed = 160.0f;
    const int numDrones = 18;

    sf::RenderWindow window(sf::VideoMode({screenWidth, screenHeight}),
                            "2D Drone Radar Simulator - C++ Sandbox");
    window.setFramerateLimit(60);
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    sf::Font font;
    if (!font.openFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
        return 1;

    std::vector<Drone> drones;
    for (int i = 0; i < numDrones; ++i) {
        Drone d;
        d.id = i;
        bool validPosition = false;
        do {
            d.x = 100.0f + static_cast<float>(std::rand() % (screenWidth - 200));
            d.y = 100.0f + static_cast<float>(std::rand() % (screenHeight - 200));
            validPosition = true;
            for (const Drone& other : drones) {
                float dx = d.x - other.x;
                float dy = d.y - other.y;
                if (dx * dx + dy * dy < 1600.0f) {
                    validPosition = false;
                    break;
                }
            }
        } while (!validPosition);

        d.vx = (static_cast<float>(std::rand() % 200) - 100.0f) * 1.2f;
        d.vy = (static_cast<float>(std::rand() % 200) - 100.0f) * 1.2f;
        drones.push_back(d);
    }

    sf::Clock clock;
    float radarAngle = 0.0f;

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
            if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
                if (key->code == sf::Keyboard::Key::Escape)
                    window.close();
            }
        }

        float dt = clock.restart().asSeconds();
        radarAngle += 1.5f * dt;

        std::vector<Acceleration> commands(drones.size());
        for (size_t i = 0; i < drones.size(); ++i) {
            if (!drones[i].crashed)
                commands[i] = computeDroneSteering(drones[i], drones);
        }

        for (size_t i = 0; i < drones.size(); ++i) {
            if (drones[i].crashed)
                continue;

            drones[i].vx += commands[i].ax * dt;
            drones[i].vy += commands[i].ay * dt;

            float speed = std::sqrt(drones[i].vx * drones[i].vx +
                                    drones[i].vy * drones[i].vy);
            if (speed > maxSpeed) {
                drones[i].vx = drones[i].vx / speed * maxSpeed;
                drones[i].vy = drones[i].vy / speed * maxSpeed;
            }

            drones[i].x += drones[i].vx * dt;
            drones[i].y += drones[i].vy * dt;

            if (drones[i].x < 30.0f) {
                drones[i].x = 30.0f;
                drones[i].vx = std::abs(drones[i].vx);
            } else if (drones[i].x > screenWidth - 30.0f) {
                drones[i].x = static_cast<float>(screenWidth - 30);
                drones[i].vx = -std::abs(drones[i].vx);
            }

            if (drones[i].y < 30.0f) {
                drones[i].y = 30.0f;
                drones[i].vy = std::abs(drones[i].vy);
            } else if (drones[i].y > screenHeight - 30.0f) {
                drones[i].y = static_cast<float>(screenHeight - 30);
                drones[i].vy = -std::abs(drones[i].vy);
            }
        }

        int activeCount = 0;
        for (size_t i = 0; i < drones.size(); ++i) {
            if (!drones[i].crashed)
                ++activeCount;

            for (size_t j = i + 1; j < drones.size(); ++j) {
                if (drones[i].crashed || drones[j].crashed)
                    continue;

                float dx = drones[i].x - drones[j].x;
                float dy = drones[i].y - drones[j].y;
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist < droneRadius * 2.0f) {
                    drones[i].crashed = true;
                    drones[j].crashed = true;
                }
            }
        }

        window.clear(sf::Color(10, 20, 15));

        sf::CircleShape ring;
        ring.setFillColor(sf::Color::Transparent);
        ring.setOutlineThickness(1.0f);
        ring.setOutlineColor(sf::Color(0, 120, 60, 100));
        for (int r = 150; r <= 450; r += 150) {
            ring.setRadius(static_cast<float>(r));
            ring.setOrigin({static_cast<float>(r), static_cast<float>(r)});
            ring.setPosition({screenWidth / 2.0f, screenHeight / 2.0f});
            window.draw(ring);
        }

        sf::RectangleShape sweep({450.0f, 2.0f});
        sweep.setFillColor(sf::Color(0, 255, 100, 80));
        sweep.setPosition({screenWidth / 2.0f, screenHeight / 2.0f});
        sweep.setRotation(sf::radians(radarAngle));
        window.draw(sweep);

        for (const Drone& d : drones) {
            sf::CircleShape shape(d.crashed ? droneRadius + 4.0f : droneRadius);
            shape.setOrigin({shape.getRadius(), shape.getRadius()});
            shape.setPosition({d.x, d.y});
            shape.setFillColor(d.crashed ? sf::Color::Red : sf::Color::Green);
            window.draw(shape);

            if (d.crashed) {
                sf::Text label(font, "CRASH", 10);
                label.setFillColor(sf::Color::Red);
                label.setPosition({d.x - 18.0f, d.y - 22.0f});
                window.draw(label);
            } else {
                sf::Vertex line[] = {
                    sf::Vertex{{d.x, d.y}, sf::Color::Yellow},
                    sf::Vertex{{d.x + d.vx * 0.25f, d.y + d.vy * 0.25f},
                               sf::Color::Yellow}
                };
                window.draw(line, 2, sf::PrimitiveType::Lines);
            }
        }

        sf::Text activeText(font,
                            "Active Drones: " + std::to_string(activeCount) +
                            " / " + std::to_string(numDrones),
                            20);
        activeText.setFillColor(sf::Color::White);
        activeText.setPosition({20.0f, 20.0f});
        window.draw(activeText);

        sf::Text instruction(font, "Edit drone_controller.hpp to control the drones!", 16);
        instruction.setFillColor(sf::Color(200, 200, 200));
        instruction.setPosition({20.0f, 48.0f});
        window.draw(instruction);

        window.display();
    }

    return 0;
}
