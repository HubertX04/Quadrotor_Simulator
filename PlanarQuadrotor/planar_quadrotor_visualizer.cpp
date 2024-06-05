#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];
    int szerokosc=150;
    int wysokosc = 15;

    int smiglo_width = 30;
    int smiglo_height = 5;
    int przesuniecie_smigla = -12;

    float cos_theta = cos(q_theta);
    float sin_theta = sin(q_theta);

    SDL_Point body[4] = {
        {(int)(q_x + (-szerokosc / 2 * cos_theta - -wysokosc / 2 * sin_theta)), (int)(q_y + (-szerokosc / 2 * sin_theta + -wysokosc / 2 * cos_theta))},
        {(int)(q_x + (szerokosc / 2 * cos_theta - -wysokosc / 2 * sin_theta)), (int)(q_y + (szerokosc / 2 * sin_theta + -wysokosc / 2 * cos_theta))},
        {(int)(q_x + (szerokosc / 2 * cos_theta - wysokosc / 2 * sin_theta)), (int)(q_y + (szerokosc / 2 * sin_theta + wysokosc / 2 * cos_theta))},
        {(int)(q_x + (-szerokosc / 2 * cos_theta - wysokosc / 2 * sin_theta)), (int)(q_y + (-szerokosc / 2 * sin_theta + wysokosc / 2 * cos_theta))}
    };

    Sint16 vx[4]= {
        body[0].x,body[1].x,body[2].x,body[3].x,
    };

    Sint16 vy[4]= {
        body[0].y,body[1].y,body[2].y,body[3].y,
    };

    SDL_SetRenderDrawColor(gRenderer.get(), 0x17, 0x3F, 0x5F, 0xFF);

    //SDL_RenderDrawLines(gRenderer.get(), body, 4);
    //SDL_RenderDrawLine(gRenderer.get(), body[3].x, body[3].y, body[0].x, body[0].y);

    filledPolygonColor(gRenderer.get(), vx, vy, 4, 0x5F3F17FF);

    filledCircleColor(gRenderer.get(), q_x, q_y, 12, 0xFF5533FF);

    int lewo_smiglo_x = (int)(q_x + (-szerokosc / 2 * cos_theta - przesuniecie_smigla * sin_theta));
    int lewo_smiglo_y = (int)(q_y + (-szerokosc / 2 * sin_theta + przesuniecie_smigla * cos_theta));
    int prawo_smiglo_x = (int)(q_x + (szerokosc / 2 * cos_theta - przesuniecie_smigla * sin_theta));
    int prawo_smiglo_y = (int)(q_y + (szerokosc / 2 * sin_theta + przesuniecie_smigla * cos_theta));

    static float kat = 0.0f;

    kat += 0.05f;
    if (kat > 2 * M_PI) {
        kat = 0.0f;
    }

    // Draw left propeller
    filledEllipseColor(gRenderer.get(), lewo_smiglo_x-sin(kat)*15, lewo_smiglo_y, smiglo_width, smiglo_height, 0xFF74FF33);
    filledEllipseColor(gRenderer.get(), lewo_smiglo_x+sin(kat)*15, lewo_smiglo_y, smiglo_width, smiglo_height, 0xFF00E9E9);

    // Draw right propeller
    filledEllipseColor(gRenderer.get(), prawo_smiglo_x-cos(kat)*15, prawo_smiglo_y, smiglo_width, smiglo_height, 0xFF74FF33);
    filledEllipseColor(gRenderer.get(), prawo_smiglo_x+cos(kat)*15, prawo_smiglo_y, smiglo_width, smiglo_height, 0xFF00E9E9);

    filledCircleColor(gRenderer.get(), lewo_smiglo_x, lewo_smiglo_y, 10, 0xFFA54044);
    filledCircleColor(gRenderer.get(), prawo_smiglo_x, prawo_smiglo_y, 10, 0xFFA54044);

    //*************** Rysowanie kropki - zaznaczenia ********************
    Eigen::VectorXf goal = quadrotor_ptr->Getgoal();

    float g_x,g_y;

    g_x = goal[0];
    g_y = goal[1];

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);
    filledCircleColor(gRenderer.get(), g_x, g_y, 5, 0xFF000000);

    //*******************************************************************


/*
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);
    filledCircleColor(gRenderer.get(), q_x, q_y, 20, 0xFF0000FF);
    SDL_RenderFillRect(gRenderer.get(), new SDL_Rect{int(q_x-szerokosc/2), int(q_y-wysokosc/2), szerokosc, wysokosc});
*/

}