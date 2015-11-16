#include "drawing.h"
#include <cmath>
#include <GL/gl.h>
using namespace std;
Drawing::Drawing()
{

}

void Drawing::drawSphere(double xpos, double ypos, double zpos, double r, int lats, int longs)
{
    int i, j;
    for(i = 0; i <= lats; i++) {
        double lat0 = M_PI * (-0.5 + (double) (i - 1) / lats);
        double z0  = sin(lat0);
        double zr0 =  cos(lat0);

        double lat1 = M_PI * (-0.5 + (double) i / lats);
        double z1 = sin(lat1);
        double zr1 = cos(lat1);

        glBegin(GL_QUAD_STRIP);
        for(j = 0; j <= longs; j++) {
            double lng = 2 * M_PI * (double) (j - 1) / longs;
            double x = cos(lng);
            double y = sin(lng);

            glNormal3f(/*xpos + */x * zr0, /*ypos + */y * zr0, /*zpos + */z0);
            glVertex3f(/*xpos + */x * zr0, /*ypos + */y * zr0, /*zpos + */z0);
            glNormal3f(/*xpos + */x * zr1, /*ypos + */y * zr1, /*zpos + */z1);
            glVertex3f(/*xpos + */x * zr1, /*ypos + */y * zr1, /*zpos + */z1);
        }
        glEnd();
    }
}
