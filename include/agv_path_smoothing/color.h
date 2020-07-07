#ifndef AGV_PATH_SMOOTHING_COLOR_H_
#define AGV_PATH_SMOOTHING_COLOR_H_

#include <std_msgs/ColorRGBA.h>

namespace agv_visualization 
{
    class Color : public std_msgs::ColorRGBA 
    {
        public:
            Color() : std_msgs::ColorRGBA() {}
            Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
            Color(double red, double green, double blue, double alpha) : Color() {
                r = red;
                g = green;
                b = blue;
                a = alpha;
            }

            static const Color White() { return Color(1.0, 1.0, 1.0); }
            static const Color Black() { return Color(0.0, 0.0, 0.0); }
            static const Color Gray() { return Color(0.5, 0.5, 0.5); }
            static const Color Red() { return Color(1.0, 0.0, 0.0); }
            static const Color Green() { return Color(0.0, 1.0, 0.0); }
            static const Color Blue() { return Color(0.0, 0.0, 1.0); }
            static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
            static const Color Orange() { return Color(1.0, 0.5, 0.0); }
            static const Color Purple() { return Color(0.5, 0.0, 1.0); }
            static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
            static const Color Teal() { return Color(0.0, 1.0, 1.0); }
            static const Color Pink() { return Color(1.0, 0.0, 0.5); }
    };
}
#endif //AGV_PATH_SMOOTHING_COLOR_H_