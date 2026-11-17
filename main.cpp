#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

// Режими роботи автомобіля
enum class DriveMode {
    Coast,      // Вільний рух (нейтраль)
    Accelerate, // Розгін (діє сила тяги)
    Brake       // Гальмування (діє сила гальмування)
};

// Стан фізичної моделі автомобіля
struct CarModel {
    const float mass = 1000.0f;       // Маса автомобіля (кг)
    float tractionForce = 3000.0f;    // Сила тяги двигуна (Н)
    float brakingForce = 4000.0f;     // Сила гальмування (Н)

    float positionX = 0.0f;           // Координата автомобіля x (м)
    float velocity = 0.0f;            // Поточна швидкість v (м/с)
    float distance = 0.0f;            // Загальний пройдений шлях S (м)
    float acceleration = 0.0f;        // Поточне прискорення a (м/с^2)
    float elapsedTime = 0.0f;         // Час моделювання (с)

    DriveMode mode = DriveMode::Accelerate;
    bool isPaused = false;

    // Показники для обчислювального експерименту
    const float targetSpeed = 20.0f;  // Цільова швидкість розгону: 20 м/с (72 км/год)
    float accelTimeToTarget = -1.0f;  // Час розгону від 0 до 20 м/с (с)

    bool isBrakingMeasured = false;
    float brakeStartPos = 0.0f;       // Координата початку гальмування (м)
    float brakeStartSpeed = 0.0f;     // Швидкість на початку гальмування (м/с)
    float brakingDistance = -1.0f;    // Виміряний гальмівний шлях (м)
};

// Скидання моделі у початковий стан
void resetSimulation(CarModel& car) {
    car.positionX = 0.0f;
    car.velocity = 0.0f;
    car.distance = 0.0f;
    car.acceleration = 0.0f;
    car.elapsedTime = 0.0f;
    car.mode = DriveMode::Accelerate;
    car.isPaused = false;
    car.accelTimeToTarget = -1.0f;
    car.isBrakingMeasured = false;
    car.brakeStartPos = 0.0f;
    car.brakeStartSpeed = 0.0f;
    car.brakingDistance = -1.0f;
}

// 1. Оброблення введення користувача
void handleInput(sf::RenderWindow& window, CarModel& car) {
    while (const std::optional event = window.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window.close();
        }

        if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
            // Зміна сили тяги (Стрілки Вгору / Вниз)
            if (key->code == sf::Keyboard::Key::Up) {
                car.tractionForce += 500.0f;
            }
            if (key->code == sf::Keyboard::Key::Down) {
                car.tractionForce -= 500.0f;
            }

            // Зміна сили гальмування (Стрілки Вправо / Вліво)
            if (key->code == sf::Keyboard::Key::Right) {
                car.brakingForce += 500.0f;
            }
            if (key->code == sf::Keyboard::Key::Left) {
                car.brakingForce -= 500.0f;
            }

            // Обмеження параметрів у допустимих межах
            car.tractionForce = std::clamp(car.tractionForce, 500.0f, 10000.0f);
            car.brakingForce = std::clamp(car.brakingForce, 500.0f, 12000.0f);

            // Перемикання режимів руху: 1 - Розгін, 2 - Гальмування, 0 - Нейтраль
            if (key->code == sf::Keyboard::Key::Num1) {
                car.mode = DriveMode::Accelerate;
                car.isBrakingMeasured = false;
            }
            if (key->code == sf::Keyboard::Key::Num2) {
                if (car.mode != DriveMode::Brake && car.velocity > 0.01f) {
                    car.isBrakingMeasured = true;
                    car.brakeStartPos = car.positionX;
                    car.brakeStartSpeed = car.velocity;
                    car.brakingDistance = -1.0f;
                }
                car.mode = DriveMode::Brake;
            }
            if (key->code == sf::Keyboard::Key::Num0) {
                car.mode = DriveMode::Coast;
                car.isBrakingMeasured = false;
            }

            // Пауза / Продовження (Пробіл) та Повторний запуск (R)
            if (key->code == sf::Keyboard::Key::Space) {
                car.isPaused = !car.isPaused;
            }
            if (key->code == sf::Keyboard::Key::R) {
                resetSimulation(car);
            }
        }
    }
}

// 2. Оновлення фізичної моделі за крок часу dt
void updatePhysics(CarModel& car, float dt) {
    if (car.isPaused) {
        return;
    }

    car.elapsedTime += dt;

    // Визначення рівнодійної сили та прискорення за другим законом Ньютона: a = F / m
    if (car.mode == DriveMode::Accelerate) {
        car.acceleration = car.tractionForce / car.mass;
    } else if (car.mode == DriveMode::Brake) {
        if (car.velocity > 0.0f) {
            car.acceleration = -car.brakingForce / car.mass;
        } else {
            car.acceleration = 0.0f;
        }
    } else {
        car.acceleration = 0.0f;
    }

    // Оновлення швидкості: v_{n+1} = v_n + a * dt
    car.velocity += car.acceleration * dt;

    // Автомобіль не рухається назад при гальмуванні до повної зупинки
    if (car.velocity < 0.0f) {
        car.velocity = 0.0f;
        car.acceleration = 0.0f;
    }

    // Оновлення координати та пройденого шляху: x_{n+1} = x_n + v_{n+1} * dt
    float deltaS = car.velocity * dt;
    car.positionX += deltaS;
    car.distance += deltaS;

    // Фіксація часу розгону до цільової швидкості (20 м/с)
    if (car.mode == DriveMode::Accelerate &&
        car.accelTimeToTarget < 0.0f &&
        car.velocity >= car.targetSpeed) {
        car.accelTimeToTarget = car.elapsedTime;
    }

    // Фіксація гальмівного шляху при повній зупинці
    if (car.isBrakingMeasured && car.velocity == 0.0f) {
        car.brakingDistance = car.positionX - car.brakeStartPos;
        car.isBrakingMeasured = false;
    }
}

// 3. Відображення сцени та параметрів моделі
void drawScene(sf::RenderWindow& window, const CarModel& car, const sf::Font& font) {
    window.clear(sf::Color(30, 35, 45));

    const float roadTopY = 440.0f;
    const float pixelsPerMeter = 10.0f; // Масштаб: 10 пікселів = 1 метр

    // Дорожнє полотно
    sf::RectangleShape road({1000.0f, 160.0f});
    road.setPosition({0.0f, roadTopY});
    road.setFillColor(sf::Color(70, 70, 75));
    window.draw(road);

    // Дорожня розмітка та кілометрові/метрові стовпчики (кожні 10 м = 100 px)
    float cameraOffset = std::fmod(car.positionX * pixelsPerMeter, 100.0f);
    for (int i = -1; i <= 11; ++i) {
        float stripeX = i * 100.0f - cameraOffset;
        sf::RectangleShape stripe({50.0f, 6.0f});
        stripe.setPosition({stripeX, roadTopY + 75.0f});
        stripe.setFillColor(sf::Color(220, 220, 220));
        window.draw(stripe);
    }

    // Візуальне положення автомобіля на екрані (циклічний проїзд по ширині вікна)
    float screenCarX = std::fmod(80.0f + car.positionX * pixelsPerMeter, 860.0f);

    // Кузов автомобіля
    sf::RectangleShape carBody({120.0f, 36.0f});
    carBody.setPosition({screenCarX, roadTopY - 50.0f});
    if (car.mode == DriveMode::Accelerate) {
        carBody.setFillColor(sf::Color(50, 180, 90));   // Зелений при розгоні
    } else if (car.mode == DriveMode::Brake) {
        carBody.setFillColor(sf::Color(220, 60, 60));   // Червоний при гальмуванні
    } else {
        carBody.setFillColor(sf::Color(70, 150, 230));  // Синій на нейтралі
    }
    window.draw(carBody);

    // Кабіна автомобіля
    sf::RectangleShape carCabin({64.0f, 24.0f});
    carCabin.setPosition({screenCarX + 24.0f, roadTopY - 74.0f});
    carCabin.setFillColor(sf::Color(180, 220, 250));
    window.draw(carCabin);

    // Колеса автомобіля
    sf::CircleShape wheelLeft(14.0f);
    wheelLeft.setPosition({screenCarX + 14.0f, roadTopY - 28.0f});
    wheelLeft.setFillColor(sf::Color(20, 20, 20));
    window.draw(wheelLeft);

    sf::CircleShape wheelRight(14.0f);
    wheelRight.setPosition({screenCarX + 78.0f, roadTopY - 28.0f});
    wheelRight.setFillColor(sf::Color(20, 20, 20));
    window.draw(wheelRight);

    // Формування тексту параметрів моделі
    std::string modeStr = "COAST (0)";
    if (car.mode == DriveMode::Accelerate) modeStr = "ACCELERATE (1)";
    else if (car.mode == DriveMode::Brake) modeStr = "BRAKE (2)";

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "State: " << (car.isPaused ? "PAUSED [Space]" : "RUNNING [Space]")
        << "   |   Mode: " << modeStr << "\n"
        << "Mass (m): " << car.mass << " kg\n"
        << "Traction Force (F_trac): " << car.tractionForce << " N   [Up / Down]\n"
        << "Braking Force  (F_brak): " << car.brakingForce << " N   [Right / Left]\n"
        << "------------------------------------------------------------\n"
        << "Position (x): " << car.positionX << " m   |   Distance (S): " << car.distance << " m\n"
        << "Velocity (v): " << car.velocity << " m/s (" << (car.velocity * 3.6f) << " km/h)\n"
        << "Acceleration (a): " << car.acceleration << " m/s^2   |   Time (t): " << car.elapsedTime << " s\n"
        << "------------------------------------------------------------\n"
        << "Accel time (0 -> 20 m/s): "
        << (car.accelTimeToTarget >= 0.0f ? std::to_string(car.accelTimeToTarget) + " s" : "measuring...") << "\n"
        << "Braking distance (to 0):  "
        << (car.brakingDistance >= 0.0f ? std::to_string(car.brakingDistance) + " m (from " + std::to_string(car.brakeStartSpeed) + " m/s)" : "press [2] while moving") << "\n"
        << "Controls: [1] Accel | [2] Brake | [0] Coast | [Space] Pause | [R] Restart";

    sf::Text hudText(font, oss.str(), 18);
    hudText.setFillColor(sf::Color::White);
    hudText.setPosition({20.0f, 15.0f});
    window.draw(hudText);

    window.display();
}

int main() {
    sf::RenderWindow window(
        sf::VideoMode({1000, 600}),
        "SFML Physics Lab - Car Motion (Variant 1)"
    );
    window.setFramerateLimit(60);

    sf::Font font;
    if (!font.openFromFile("../resources/fonts/arial.ttf") &&
        !font.openFromFile("../../resources/fonts/arial.ttf") &&
        !font.openFromFile("resources/fonts/arial.ttf")) {
        return 1;
    }

    CarModel car;
    sf::Clock clock;

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        // Обмеження кроку часу dt для стабільності чисельного інтегрування
        if (dt > 0.05f) {
            dt = 0.05f;
        }

        handleInput(window, car);
        updatePhysics(car, dt);
        drawScene(window, car, font);
    }

    return 0;
}