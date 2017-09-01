// Import Serial Things
/*#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>

#include <map>
#include <vector>

// ------ DEFINE COMS PROTOCOL ------
const int MOVE_LENGTH_REGISTER_ADDRESS = 13;
const int UPDATE_POSITION_REGISTER_ADDRESS = 15;
const int RUN_PROGRAM_REGSITER_ADDRESS = 17;
const int LIGHT_COLOR_REGSITER_ADDRESS = 19;

const int CLEAR_CTRL_DEQ_PROGRAM = 0;
const int RANDOM_GAZE_PROGRAM = 1;
const int REMOTE_CONTROL_PROGRAM = 2;

const double CTRL_FREQ = 550.; // Embedded controller loop rate in Hz
const int NUM_DOFS = 12;
const double CONFIRM_TIMEOUT = 60./CTRL_FREQ;  // Send commands at 10 Hz


int main( int    argc,      char** argv  )
{
     //
     // Open the serial port.
     //
     using namespace std;
     using namespace LibSerial ;
     SerialStream serial_port ;
     char c;
     serial_port.Open( "/dev/ttyACM0" ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                   << "Error: Could not open serial port."
                   << std::endl ;
         exit(1) ;
     }
     //
     // Set the baud rate of the serial port.
     //
     serial_port.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "Error: Could not set the baud rate." <<  std::endl ;
         exit(1) ;
     }
     //
     // Set the number of data bits.
     //
     serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "Error: Could not set the character size." <<  
std::endl ;
         exit(1) ;
     }
     //
     // Disable parity.
     //
     serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "Error: Could not disable the parity." <<  
std::endl ;
         exit(1) ;
     }
     //
     // Set the number of stop bits.
     //
     serial_port.SetNumOfStopBits( 1 ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "Error: Could not set the number of stop bits."
                   << std::endl ;
         exit(1) ;
     }
     //
     // Turn off hardware flow control.
     //
     serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
     if ( ! serial_port.good() )
     {
         std::cerr << "Error: Could not use hardware flow control."
                   << std::endl ;
         exit(1) ;
     }
     //
     // Do not skip whitespace characters while reading from the
     // serial port.
     //
     // serial_port.unsetf( std::ios_base::skipws ) ;
     //
     // Wait for some data to be available at the serial port.
     //
     //
     // Keep reading data from serial port and print it to the screen.
     //
  // Wait for some data to be available at the serial port.
     //
     while( serial_port.rdbuf()->in_avail() == 0 )
     {
         usleep(100) ;
     }


     char out_buf[] = "check";
     serial_port.write(out_buf, 5);  <-- FIRST COMMAND
     while( 1  )
     {
         char next_byte;
         serial_port.get(next_byte);  HERE I RECEIVE THE FIRST ANSWER
         std::cerr << next_byte;

     }
     std::cerr << std::endl ;
     return EXIT_SUCCESS ;
}*/