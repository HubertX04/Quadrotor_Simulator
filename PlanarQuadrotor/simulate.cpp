/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <thread>
#include <matplot/matplot.h>

Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 0.004, 0.004, 400, 0.008, 0.045, 2 / 2 / M_PI;
    R.row(0) << 30, 7;
    R.row(1) << 7, 30;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}
void doPlot(std::vector<float> x_history,std::vector<float> y_history,std::vector<float> theta_history,std::vector<float> time){

    matplot::subplot(3,1,0);
    matplot::plot(time,x_history);
    matplot::title("X");
    matplot::subplot(3,1,1);
    matplot::plot(time,y_history);
    matplot::title("Y");
    matplot::subplot(3,1,2);
    matplot::plot(time,theta_history);
    matplot::title("Theta");
    matplot::show();
    
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;
    const int start_x=350;
    const int start_y=350;


    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    initial_state << start_x, start_y, 0, 0, 0, 0; 
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.003;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    std::vector<float> x_history{float(start_x)};
    std::vector<float> y_history{float(start_y)};
    std::vector<float> theta_history{0};
    std::vector<float> time{0};
    

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        float a=0;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else 
                {
                    if (e.type == SDL_MOUSEMOTION)
                    {
                        SDL_GetMouseState(&x, &y);
                        std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                    }
                    if(e.type == SDL_MOUSEBUTTONDOWN){
                        SDL_GetMouseState(&x, &y);
                        goal_state<<x,y,0,0,0,0;
                        std::cout<<"Mouse left-click "<<x<<" "<<y<<std::endl;
                        quadrotor.SetGoal(goal_state);
                    }
                    else if(e.type==SDL_KEYDOWN && e.key.keysym.sym==SDLK_p){
                        std::thread p(doPlot,x_history,y_history,theta_history,time);
                        p.detach();
                    }
                }
            }

            SDL_Delay((int) dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
            while(abs(x_history.back()-quadrotor.GetState()[0])>0.2 || abs(y_history.back()-quadrotor.GetState()[1])>0.2){
                if(a>10000){
                    //removes from the vector
                     time.erase(time.begin());
                     x_history.erase(x_history.begin());
                     y_history.erase(y_history.begin());
                     theta_history.erase(theta_history.begin());
                     }
            time.push_back(a++);
            x_history.push_back(quadrotor.GetState()[0]);
            y_history.push_back(quadrotor.GetState()[1]);
            theta_history.push_back(quadrotor.GetState()[2]);
            
            }
        
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}