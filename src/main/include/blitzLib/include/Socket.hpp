#pragma once

#include <frc/WPILib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>

using namespace std;
using namespace frc;

namespace Blitz
{
    class Socket
    {
        public:
            Socket(std::string *output)
            {
                this->output = output;
            }

            void Open();

            std::string *output;

            
        private:

            static void * Server(void * args);
            
            pthread_t threads;
            
    };  
}