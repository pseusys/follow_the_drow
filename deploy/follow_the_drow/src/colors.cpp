#include <stdexcept>
#include <algorithm>

#include "follow_the_drow/misc.hpp"

#include "colors.hpp"


const std::string getStringFromColor(const Color color) {
    switch (color) {
        case Color::Black: return "Black";
        case Color::Blue: return "Blue";
        case Color::Green: return "Green";
        case Color::Cyan: return "Cyan";
        case Color::Red: return "Red";
        case Color::Magenta: return "Magenta";
        case Color::Yellow: return "Yellow";
        case Color::White: return "White";
        default: throw std::runtime_error("Bad Color value!");
    }
}

const Color getColorFromString(const std::string string) {
    std::string lower = lowercase(string);
    if (lower == "black") return Color::Black;
    if (lower == "blue") return Color::Blue;
    if (lower == "green") return Color::Green;
    if (lower == "cyan") return Color::Cyan;
    if (lower == "red") return Color::Red;
    if (lower == "magenta") return Color::Magenta;
    if (lower == "yellow") return Color::Yellow;
    if (lower == "white") return Color::White;
    throw std::runtime_error("Bad color name '" + string + "'!");
}

const std_msgs::ColorRGBA getRGBAFromColor(const Color color) {
    std_msgs::ColorRGBA rgba;
    rgba.r = 0;
    rgba.g = 0;
    rgba.b = 0;
    rgba.a = 1;
    switch (color) {
        case Color::Black:
            break;
        case Color::Blue:
            rgba.b = 1;
            break;
        case Color::Green:
            rgba.g = 1;
            break;
        case Color::Cyan:
            rgba.b = 1;
            rgba.g = 1;
            break;
        case Color::Red:
            rgba.r = 1;
            break;
        case Color::Magenta:
            rgba.r = 1;
            rgba.b = 1;
            break;
        case Color::Yellow:
            rgba.r = 1;
            rgba.g = 1;
            break;
        case Color::White:
            rgba.r = 1;
            rgba.g = 1;
            rgba.b = 1;
            break;
        default:
            throw std::runtime_error("Bad Color value!");
    }
    return rgba;
}
