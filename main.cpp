#include <opencv2/opencv.hpp>
#include <SFML/Graphics.hpp>
#include <thread>
#include <deque>

const int FILTER_SIZE = 10;
std::deque<double> redFilterBuffer;
double redY = 0.0;
bool redDetected = false;

double applyFilter(std::deque<double>& filterBuffer, double newValue) {
    filterBuffer.push_back(newValue);
    if (filterBuffer.size() > FILTER_SIZE) {
        filterBuffer.pop_front();
    }

    double sum = 0;
    for (double value : filterBuffer) {
        sum += value;
    }
    return sum / filterBuffer.size();
}

void trackingCor() {
    cv::VideoCapture cap(0);

    cv::Mat frame, hsv, maskred;

    while (true) {
        cap >> frame;

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar lowerred(80, 70, 70);
        cv::Scalar upperred(100, 255, 255);

        cv::inRange(hsv, lowerred, upperred, maskred);

        std::vector<std::vector<cv::Point>> contoursred;
        cv::findContours(maskred, contoursred, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contoursred.empty()) {
            double largestredArea = 0;
            int largestredContourIndex = -1;

            for (int i = 0; i < contoursred.size(); i++) {
                double area = cv::contourArea(contoursred[i]);
                if (area > largestredArea) {
                    largestredArea = area;
                    largestredContourIndex = i;
                }
            }

            if (largestredContourIndex != -1) {
                cv::Rect boundingBoxred = cv::boundingRect(contoursred[largestredContourIndex]);
                double detectedredY = boundingBoxred.y + boundingBoxred.height / 2;
                redY = applyFilter(redFilterBuffer, detectedredY);
                redDetected = true;
                cv::rectangle(frame, boundingBoxred, cv::Scalar(255, 0, 0), 2);
            }
        } else {
            redDetected = false;
        }


        cv::imshow("Camera", frame);

        if (cv::waitKey(30) == 27) {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
}

float customLerp(float start, float end, float t) {
    return start + t * (end - start);
}

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Pong Game");

    sf::RectangleShape player1(sf::Vector2f(15, 120));
    player1.setPosition(50, 250);
    player1.setFillColor(sf::Color::Cyan);

    sf::RectangleShape player2(sf::Vector2f(15, 120));
    player2.setPosition(740, 250);
    player2.setFillColor(sf::Color::Red);

    sf::CircleShape ball(12);
    ball.setFillColor(sf::Color::White);
    ball.setPosition(395, 295);

    float ballSpeedX = 0.2f;
    float ballSpeedY = 0.2f;
    float smoothFactor = 0.1f;

    std::thread trackingThread(trackingCor);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float targetY1 = player1.getPosition().y;
        if (redDetected && redY >= 90 && redY <= 380) {
            targetY1 = (redY - 90) * (480.0 / (380.0 - 90.0));
        }
        player1.setPosition(50, customLerp(player1.getPosition().y, targetY1, smoothFactor));

        ball.move(ballSpeedX, ballSpeedY);

        if (ball.getPosition().y <= 0 || ball.getPosition().y >= 600 - ball.getRadius() * 2) {
            ballSpeedY = -ballSpeedY;
        }

        if (ball.getGlobalBounds().intersects(player1.getGlobalBounds()) ||
            ball.getGlobalBounds().intersects(player2.getGlobalBounds())) {
            ballSpeedX = -ballSpeedX;
            }

        if (ball.getPosition().x <= 0 || ball.getPosition().x >= 800 - ball.getRadius() * 2) {
            ball.setPosition(395, 295);
            ballSpeedX = (ballSpeedX > 0 ? 0.2f : -0.2f);
            ballSpeedY = (ballSpeedY > 0 ? 0.2f : -0.2f);
        }


        float targetY2 = ball.getPosition().y + ball.getRadius() - player2.getSize().y / 2;
        player2.setPosition(player2.getPosition().x, targetY2);

        window.clear();
        window.draw(player1);
        window.draw(player2);
        window.draw(ball);
        window.display();
    }

    trackingThread.join();
    return 0;
}