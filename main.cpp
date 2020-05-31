#include <windows.h>
#include <iomanip>

#include <gl/gl.h>
#include"ClaseOpenGL.h"

///Copyright (C) <2017>  <Eliseo Rivera> curso.tareas@gmail.com
LRESULT CALLBACK WindowProc(HWND, UINT, WPARAM, LPARAM);
void EnableOpenGL(HWND hwnd, HDC*, HGLRC*);
void DisableOpenGL(HWND, HDC, HGLRC);
vector3d d1,d2,d3,d4,d5,d6,d7;

float dtheta(0.01);
 ClaseOpenGL* Miclase=NULL;
int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow)
{
cout << fixed << setprecision(7);
    WNDCLASSEX wcex;
    HWND hwnd;
    HDC hDC;
    HGLRC hRC;
    MSG msg;
    BOOL bQuit = FALSE;


    /* register window class */
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_OWNDC;
    wcex.lpfnWndProc = WindowProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(NULL, IDI_APPLICATION);
    wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wcex.lpszMenuName = NULL;
    wcex.lpszClassName = "GLSample";
    wcex.hIconSm = LoadIcon(NULL, IDI_APPLICATION);;

Miclase = new ClaseOpenGL;  //agregar código opengl

    if (!RegisterClassEx(&wcex))
        return 0;

    /* create main window */
    hwnd = CreateWindowEx(0,
                          "GLSample",
                          "Cinemática inversa  Brazo Robótico de 6 grados de libertad ",
                          WS_OVERLAPPEDWINDOW,
                          CW_USEDEFAULT,
                          CW_USEDEFAULT,
                          800,
                          600,
                          NULL,
                          NULL,
                          hInstance,
                          NULL);
EnableOpenGL(hwnd, &hDC, &hRC); //va primero esto
ShowWindow(hwnd, nCmdShow); // y despues mostrar ventana
UpdateWindow(hwnd);					// update the window

    /* enable OpenGL for the window */

Miclase->inicializar();
SwapBuffers(hDC);
    /* program main loop */
    while (!bQuit)
    {
        /* check for messages */
        if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
        {
            /* handle or dispatch messages */
            if (msg.message == WM_QUIT)
            {
                bQuit = TRUE;
            }
            else
            {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
        }
        else
        {
            /* OpenGL animation code goes here */

            Miclase->renderizar();
            SwapBuffers(hDC);
            Miclase->preparar(0.001);

            Sleep (1);
        }
    }
delete Miclase;
Miclase=NULL;
    /* shutdown OpenGL */
    DisableOpenGL(hwnd, hDC, hRC);

    /* destroy the window explicitly */
    DestroyWindow(hwnd);

    return msg.wParam;
}

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{int height, width;
bool status;
    switch (uMsg)
    {
    case WM_CREATE:


    break;
        case WM_CLOSE:
            PostQuitMessage(0);
        break;
    case WM_SIZE:
		height = HIWORD(lParam);		// retrieve width and height
		width = LOWORD(lParam);

	Miclase->Resize(width, height);

    break;

        case WM_DESTROY:
            return 0;

        case WM_KEYDOWN:
        {

            switch (wParam)
            {



          case 'R':

                Miclase->robot.theta4=Miclase->robot.theta4+dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta4, {0,0,Miclase->robot.z4});   //b4
                Miclase->robot.THList[8]=Miclase->robot.THz;
                 cout<<" theta4  =  "<< Miclase->robot.theta4<<endl;
               Miclase->robot.  initInverseKinematics2();
                break;

                     case 'Y':

                Miclase->robot.theta4=Miclase->robot.theta4-dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta4, {0,0,Miclase->robot.z4});   //b4
                Miclase->robot.THList[8]=Miclase->robot.THz;
                cout<<" theta4  =  "<< Miclase->robot.theta4<<endl;
               Miclase->robot.  initInverseKinematics2();

                break;


                case 'F':
                Miclase->robot.theta5=Miclase->robot.theta5+dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta5, {0,0,Miclase->robot.z5});    //b5
                Miclase->robot.THList[10]=Miclase->robot.THz;
                 Miclase->robot.  initInverseKinematics2();
                break;

                     case 'H':

                Miclase->robot.theta5=Miclase->robot.theta5-dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta5, {0,0,Miclase->robot.z5});     //b5
                Miclase->robot.THList[10]=Miclase->robot.THz;
               Miclase->robot.  initInverseKinematics2();

                break;

                case 'V':
                Miclase->robot.theta6=Miclase->robot.theta6+dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta6, {0,0,Miclase->robot.z6});     //b6
                Miclase->robot.THList[12]=Miclase->robot.THz;
                 Miclase->robot.  initInverseKinematics2();
                break;

                    case 'N':

                Miclase->robot.theta6=Miclase->robot.theta6-dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta6,{0,0,Miclase->robot.z6});      //b6
                Miclase->robot.THList[12]=Miclase->robot.THz;
                 Miclase->robot.  initInverseKinematics2();
                break;

                case 'K':
                Miclase->robot.theta7=Miclase->robot.theta7-dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta7, {0,0,Miclase->robot.z7});     //b7, efector final
                Miclase->robot.THList[14]=Miclase->robot.THz;
                 Miclase->robot.  initInverseKinematics2();
                break;



          case 97:

                Miclase->robot.theta1=Miclase->robot.theta1+dtheta;
                Miclase->robot.AplicarTHz(Miclase->robot.theta1, {0,0,Miclase->robot.z1});     //b1
                Miclase->robot.THList[2]=Miclase->robot.THz;
                                Miclase->robot.  initInverseKinematics2();
                break;
            case 99:

                Miclase->robot.theta1=Miclase->robot.theta1-dtheta;
                Miclase->robot.AplicarTHz( Miclase->robot.theta1, {0,0,Miclase->robot.z1});     //b1
              Miclase->robot.THList[2]=Miclase->robot.THz;
             Miclase->robot.initInverseKinematics2();

                break;

             case 100:

                Miclase->robot.theta2=Miclase->robot.theta2+dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta2, {0,0, Miclase->robot.z2});  //b2
                Miclase->robot.THList[4]=Miclase->robot.THz;
                Miclase->robot.initInverseKinematics2();
                break;
            case 102:

                Miclase->robot.theta2=Miclase->robot.theta2-dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta2, {0,0,Miclase->robot.z2});     //b2
                Miclase->robot.THList[4]=Miclase->robot.THz;
                Miclase->robot.initInverseKinematics2();

                break;

         case 103:

                Miclase->robot.theta3=Miclase->robot.theta3+dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta3, {0,0,Miclase->robot.z3});     //b3
                Miclase->robot.THList[6]=Miclase->robot.THz;
                Miclase->robot.initInverseKinematics2();
                break;
            case 105:

                Miclase->robot.theta3=Miclase->robot.theta3-dtheta;
                Miclase->robot.AplicarTHz(   Miclase->robot.theta3,{0,0,Miclase->robot.z3});       //b3
                Miclase->robot.THList[6]=Miclase->robot.THz;
                Miclase->robot.initInverseKinematics2();
                break;

            case  'P':
            status= Miclase->robot.InverseKinematics2();
            if (status==true){
            Miclase->robot.t=Miclase->robot.t+0.2;  //actualiza el tiempo
            Miclase->robot.Parametrica(); //nueva posición de punto terminal a partir de trayectoria paramétrica
            Miclase->robot.O6=Miclase->robot.TPG-Miclase->robot.TPVG; //Calcular O6, TPG se actualiza en paramétrica
            }
            break;
           case  'E':
            Miclase->robot.Estado();  //muestra algunos parámetros de interés, como el valor de la posición del punto terminal y las variables de articulación

            break;



                case 'A':
                Miclase->thetaCamera=Miclase->thetaCamera+0.05;

                break;
                  case 'D':
                Miclase->thetaCamera=Miclase->thetaCamera-0.05;

                break;

                case 'W':
                Miclase->phiCamera=Miclase->phiCamera+0.05;

                break;
                  case 'S':
                Miclase->phiCamera=Miclase->phiCamera-0.05;

                break;
                case VK_ESCAPE:
                    PostQuitMessage(0);

                break;
            }
        }
        break;

        default:
            return DefWindowProc(hwnd, uMsg, wParam, lParam);
    }

    return 0;
}

void EnableOpenGL(HWND hwnd, HDC* hDC, HGLRC* hRC)
{
    PIXELFORMATDESCRIPTOR pfd;

    int iFormat;

    /* get the device context (DC) */
    *hDC = GetDC(hwnd);

    /* set the pixel format for the DC */
    ZeroMemory(&pfd, sizeof(pfd));

    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW |
                  PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 24;
    pfd.cDepthBits = 16;
    pfd.iLayerType = PFD_MAIN_PLANE;

    iFormat = ChoosePixelFormat(*hDC, &pfd);

    SetPixelFormat(*hDC, iFormat, &pfd);

    /* create and enable the render context (RC) */
    *hRC = wglCreateContext(*hDC);

    wglMakeCurrent(*hDC, *hRC);
}

void DisableOpenGL (HWND hwnd, HDC hDC, HGLRC hRC)
{
    wglMakeCurrent(NULL, NULL);
    wglDeleteContext(hRC);
    ReleaseDC(hwnd, hDC);
}

