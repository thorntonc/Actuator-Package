#include <iostream>

#include <csignal>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <chrono>
#include <thread>
#include <fstream>

#include "device_wrapper.h"
#ifdef _WIN32
	#include "windows.h"
#endif
#include "hold_position_example.h"
#include "open_speed_example.h"
#include "read_all_example.h"
#include "current_control.h"
//#include "findpolesexample.h"
#include "two_dev_position_example.h"
#include "leaderfollower.h"

using namespace std;
using namespace std::literals::chrono_literals;

int activeDemo = 0;

void printMenu();
char userSelection(void);

void getUserPort(const string filename, string *ports, int *baudRate);

string      configFile = "../com.txt";
const int   MAX_FLEXSEA_DEVS = 4;
int         deviceIds[MAX_FLEXSEA_DEVS];
bool        shouldQuit = false;

void my_handler(int s)
{
            (void)s;
           cout << "Caught CTRL-C, exiting...\n";
           shouldQuit = true;
}

int main()
{
    //
    // Capture the CTRL-C signal
    //
    std::signal(SIGINT, my_handler);

    cout << "Demo code - C++ project with FlexSEA-Stack DLL" << endl;


    unsigned idx = 0;
    string portName[MAX_FLEXSEA_DEVS];
    int baudRate;
    //
    // Read the COM ports from COM.TXT
    //
    getUserPort(configFile, portName, &baudRate);

    //
    // Start opening the com port s and reading the
    //  FlexSEA device information
    //
    unsigned devicesOpened = 0;
    for(idx = 0; idx < MAX_FLEXSEA_DEVS; ++idx)
    {
        //
        // Stop looking at the first empty port name
        //
        if(portName[idx].empty())
            break;

        cout << "Connecting to port: " << portName[idx] << endl;

        //
        // Open the port
        //
	int devId = fxOpen((char *)portName[idx].c_str(), baudRate, 0); 
        if (devId != -1)
	{
		cout << "Connected to device: " << devId << endl;
		deviceIds[devicesOpened++] = devId;
	}
    }

    //
    // Print the menu and ask what demo to run
    //
    printMenu();
    userSelection();

    try {

        switch(activeDemo)
        {
            case 0:
                runReadAll(deviceIds[0], &shouldQuit);
                break;
            case 1:
                runOpenSpeed(deviceIds[0], &shouldQuit);
                break;
            case 2:
                runCurrentControl(deviceIds[0], &shouldQuit);
                break;
            case 3:
                runHoldPosition(deviceIds[0], &shouldQuit);
                break;
            case 4:
       //         runFindPoles(deviceIds[0]);
                break;
            case 5:
                runTwoDevicePositionControl(deviceIds[0], deviceIds[1], &shouldQuit);
                break;
            case 6:
                runLeaderFollower(deviceIds[0], deviceIds[1], &shouldQuit);
                break;
            default:
                break;
        }

    } catch (...) {
        cout << "Crashed..." << endl;
    }

    //
    // Close all of the FlexSEA devices
    //
    fxCloseAll();

    return 0;
}

void printMenu(void)
{
    cout << "What demo would you like to try?" << endl << endl;
    cout << "[0] Read-Only: no actuator, sensors only." << endl;
    cout << "[1] Open-Speed: PWM will ramp up, then down, then up..." << endl;
    cout << "[2] Current Control" << endl;
    cout << "[3] Position Control" << endl;
    cout << "[4] Find Poles" << endl;
    cout << "[5] Two device Position Control" << endl;
    cout << "[6] Two Device Leader-Follower" << endl;
    cout << "[q] Quit program." << endl << endl;
    //ToDo: expand that list
}

char userSelection(void)
{
startOfSelection:
    char c = 0;
    cout << "Please enter your selection: ";
    cin >> c;

    if(c >= '0' && c <= '6')
    {
        //Valid demo:
        activeDemo = (int)(c - '0');
    }
    else if(c == 'q')
    {
        activeDemo = 99;
    }
    else
    {
        cout << "Invalid selection, please try again." << endl;
        goto startOfSelection;
    }

    return activeDemo;
}

void getUserPort(const string filename, string *ports, int *baudRate)
{
    cout << "Opening file: " << filename << endl;
    ifstream cfgFile( filename );

    if( ! cfgFile)
    {
        cout << endl << "No com.txt found..." << endl;
        cout << "Please copy com_template.txt to a file named com.txt" << endl;
        cout << "Be sure to use the same format of baud rate on the first line," \
            " and com ports on preceding lines" << endl;
        exit(1);
    }

    string line;
    getline(cfgFile, line);
    *baudRate = std::stoi(line);

    std::cout << "using baud rate: " << *baudRate << std::endl;

    int i = 0;
    while( ! cfgFile.eof() )
    {
        getline(cfgFile, line);
        if(! line.empty())
        {
            ports[i] = line;
            // cout << "read Com port " << line << " (" << ports[i] << ")" << endl;
            if( ++i > MAX_FLEXSEA_DEVS)
                break;
        }
    }
}
