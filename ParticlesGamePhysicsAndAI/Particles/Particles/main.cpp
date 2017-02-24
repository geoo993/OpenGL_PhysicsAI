//
//  main.cpp
//  Particles
//
//  Created by GEORGE QUENTIN on 29/01/2017.
//  Copyright © 2017 GEORGE QUENTIN. All rights reserved.
//
#include <iostream>

//GLEW
#include <GL/glew.h>

//GLFW
#include <GLFW/glfw3.h>

#include <sys/time.h>
#include <stdlib.h>

#include "Emitter.h"
#include "DrawGLScene.h"


const	float	initial_zoom = -40.0f;

int window_width = 1024;
int window_height = 768;
int keyCode; 

double get_time() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + tv.tv_usec/1000000.0;
}

void onKeyDown(int key, int action){
    
    switch (action) {
        case GLFW_PRESS:
            keyCode = key; 
            std::cout << "key pressed" << std::endl;
            break;
        case GLFW_RELEASE:
            keyCode = -1; 
            std::cout << "key released" << std::endl; 
            break;
        case GLFW_REPEAT:
            keyCode = key;
            std::cout << "key down" << std::endl; 
            break;
        default:
            break;
    }
   
}

void check_keys(Emitter &particles, float &zoom) {
  
    /*-----------------------------------------------------------------------------
     *  Every frame, check for keyboard input
     *-----------------------------------------------------------------------------*/
    
    if (keyCode!=-1){
        
        switch (keyCode) {
            case GLFW_KEY_I:
                zoom += 0.1f; // Zoom In
                break;
            case GLFW_KEY_O:
                zoom -= 0.1f; // Zoom Out
                break;
            case GLFW_KEY_UP:
                particles.speedUp(); // Increase Upward Speed
                break;
            case GLFW_KEY_DOWN:
                particles.speedDown(); // Increase Downward Speed
                break;
            case GLFW_KEY_LEFT:
                particles.speedLeft(); // Increase Speed To The Left
                break;
            case GLFW_KEY_RIGHT:
                particles.speedRight(); // Increase Speed To The Right
                break;
            default:
                break;
        }
      
    }
}


static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS){
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
    
    std::cout << "Key pressed with key: " << key << " and with action: " << action << std::endl;
    
    onKeyDown(key, action);
}

int main(int argc, const char * argv[]) {

    if( !glfwInit() ){
        exit(EXIT_FAILURE);
    }
    
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
//    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//    
//    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    
    GLFWwindow * window;       //Create Window (use glfwGetPrimaryMonitor() for fullscreen)
    window = glfwCreateWindow(window_width,window_height,"glfw",NULL,NULL);
    
    glfwSetErrorCallback(error_callback);
    
    if (!window) {                   //Check Validity
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);  //Make Window Current Context
    
    glfwSetKeyCallback(window, key_callback);
    glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);
    
    glfwMakeContextCurrent(window);
    
    glewExperimental = GL_TRUE;
    glewInit();
    
    // get version info
    const GLubyte* renderer = glGetString(GL_RENDERER); // get renderer string
    const GLubyte* version = glGetString(GL_VERSION); // version as a string
    printf("Renderer: %s\n", renderer);
    printf("OpenGL version supported %s\n", version);
    
    // tell GL to only draw onto a pixel if the shape is closer to the viewer
    glEnable(GL_DEPTH_TEST); // enable depth-testing
    glDepthFunc(GL_LESS); // depth-testing interprets a smaller value as "closer"
    
    keyCode = -1;
    double last_time = get_time();
    
    InitGL();
    ReSizeGLScene(window_width, window_height);
    
    Emitter particles;
    float	zoom = initial_zoom;		// z-coord of viewpoint

    while ( !glfwWindowShouldClose(window) ){
        
        //Set view port size and position every frame of animation
        glfwGetFramebufferSize(window, &window_width, &window_height);
        glViewport(0,0,window_width,window_height);
        glClearColor(.2,.8,.8,1);                  //<-- CLEAR WINDOW CONTENTS
        
        DrawGLScene(particles, zoom);
        
        glfwSwapBuffers(window);                //<-- SWAP BUFFERS
        double this_time = get_time();
        particles.step(float(this_time - last_time));
        last_time = this_time;
        
        check_keys(particles, zoom);
        glfwPollEvents();                       //<-- LISTEN FOR WINDOW EVENTS
    }
    
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
    

    return 0;
}
