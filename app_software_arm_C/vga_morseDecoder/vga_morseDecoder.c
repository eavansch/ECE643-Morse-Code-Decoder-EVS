/*
File: VGA.c
Modified by: Erik Van Schijndel
Course: ECE 643
Instructor: Don Gruenbacher
*/
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h> //Include for fabs()
#include "socal/hps.h" //Pull from LW Axi bridge  
#include "ARM_A9_HPS.h" //Result of sopc-create-header-files 

#define HW_REGS_BASE ( 0xFC000000 )
#define HW_OCRAM_BASE ( 0xC8000000 )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
#define FPGA_CHAR_BASE      0xC9000000

#define PHYSMEM_32(addr) (*((unsigned int *)(virtual_base + (addr & HW_REGS_MASK))))
#define PHYSMEM_16(addr) (*((unsigned short *)(virtual_base + (addr & HW_REGS_MASK))))

#define MAX_BUFFER_SIZE 256 // Buffer size for text_buffer

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
void VGA_box(int x1, int y1, int x2, int y2, short pixel_color, void *virtual_base)
{ 
	unsigned int pixel_ptr, row, col;

	/* assume that the box coordinates are valid */
	for (row = y1; row <= y2; row++)
		for (col = x1; col <= x2; ++col)
		{
			//pixel_ptr = HW_OCRAM_BASE + (row << 10) + (col << 1); // Original versio
			pixel_ptr = HW_OCRAM_BASE + (row << 10) + col; // New one for 640x480
			PHYSMEM_16(pixel_ptr) = pixel_color;		// set pixel color
		}
}


void VGA_text(int x, int y, char * text_ptr, void *virtual_base)
{
	int offset;
  	unsigned int  char_ptr;

	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		char_ptr =  FPGA_CHAR_BASE + offset;
		
		PHYSMEM_32(char_ptr) = *(text_ptr);
		
		++text_ptr;
		++offset;
	}
}

void VGA_text_clear(void *virtual_base)
{
	int x,y;
	int offset;
	char blank[1] = " ";
  	unsigned int  char_ptr;

	// Character coordinats are from 0,0 to 79,59 (x,y) position
	for(x = 0; x < 80; x++)
		for(y = 0; y < 60; y++)
		{
		/* assume that the text string fits on one line */
		offset = (y << 7) + x;
		char_ptr =  FPGA_CHAR_BASE + offset;
		PHYSMEM_32(char_ptr) = *blank;
		}
}

//Clear screen
void VGA_clear(void *virtual_base)
{
	unsigned int pixel_ptr, row, col;

	/* assume that the box coordinates are valid */
	for (row = 1; row <= 480; row++)
		for (col = 0; col <= 640; ++col)
		{
			//pixel_ptr = HW_OCRAM_BASE + (row << 10) + (col << 1); // Original versio
			pixel_ptr = HW_OCRAM_BASE + (row << 10) + col; // New one for 640x480
			PHYSMEM_16(pixel_ptr) = 0x0000;		// set pixel color to black
		}
	
	VGA_text_clear(virtual_base);	
}	

//Function to draw line and calculate slope between two coordinate pairs (Lab5)
void VGA_line(int x1, int y1, int x2, int y2, short pixel_color, void *virtual_base)
{
	unsigned int pixel_ptr, row, col;
	int x, y;
	float m, step;
	
	if(x1 == x2) // 0 slope
	{
		if(y1 > y2) { int temp = y1; y1 = y2; y2 = temp; } // Ensure y1 <= y2
		for (row = y1; row <= y2; row++)
		{
			pixel_ptr = HW_OCRAM_BASE + (row << 10) + x1; // New one for 640x480
			PHYSMEM_16(pixel_ptr) = pixel_color;		// set pixel color
		}
	}
	else if(y1 == y2) // Inf
	{
			if(x1 > x2) { int temp = x1; x1 = x2; x2 = temp; } // Ensure x1 <= x2
			for (col = x1; col <= x2; col++)
			{
			pixel_ptr = HW_OCRAM_BASE + (y1 << 10) + col; // New one for 640x480
			PHYSMEM_16(pixel_ptr) = pixel_color;		// set pixel color
			}
	}
	else // Calculate slope
	{
		m = ((1.0)*(y2-y1))/((1.0)*(x2-x1));
		
		// Absolute value of slope is less than or equal to 1
		if(fabs(m) <= 1) // fabs() = float abs()
		{
			if(x1 > x2) // Ensure x2 > x1
			{
				int tempX = x1, tempY = y1;
				x1 = x2; 
				y1 = y2;
				x2 = tempX;
				y2 = tempY;
			}
			y = y1;
			for ( x = x1; x <= x2; x++)
			{
				pixel_ptr = HW_OCRAM_BASE + (y << 10) + x; // New one for 640x480
				PHYSMEM_16(pixel_ptr) = pixel_color;		// set pixel color			
				step += m;
				if(fabs(step) >= 0.5)
				{
				y += (m > 0 ? 1 : -1); // Increment or decrement y based on the slope's sign
				step -= (m > 0 ? 1 : -1); // Increment/decrement steps based on slope sign
				}
			}
		}
		// slope is greater than 1
		else
		{
			if(y1 > y2) // Swap coordinates to ensure y2 > y1
			{
				int tempX = x1, tempY = y1;
				x1 = x2; 
				y1 = y2;
				x2 = tempX;
				y2 = tempY;
			}
			x = x1;
			for (y = y1; y <= y2; y++)
			{
				pixel_ptr = HW_OCRAM_BASE + (y << 10) + x; // New one for 640x480
				PHYSMEM_16(pixel_ptr) = pixel_color;		// set pixel color
				step += 1/m;
				if(fabs(step) >= 0.5)
				{
				x += (m > 0 ? 1 : -1); // Increment or decrement x based on the slope's sign
				step -= (m > 0 ? 1 : -1);
				}
			}	
		}
	}
}
// Function to draw an underline
void VGA_draw_underline(int x, int y, bool visible, void *virtual_base) {
    unsigned int pixel_ptr; 
    short pixel_color = visible ? 0xFFFF : 0x0000; // White for visible, Black for invisible
    int i = 0; // Fixed length of 5 pixels
    for (i = 0; i < 5; ++i) { //Underline length of 5 pixels
        pixel_ptr = HW_OCRAM_BASE + ((y + 1) << 10) + (x + i); // Underline is drawn one pixel below the character
        PHYSMEM_16(pixel_ptr) = pixel_color;
    }
}

// Test program for use with the DE1-SoC University Computer
// 

int main(int argc,char ** argv) {
	
    void *virtual_base;
    int fd;
	
	void *h2p_lw_letter_decoded_addr; // Ptr to PIO memory location
	
	int letter_decoded = -1;
	int prev_letter_decoded = -1;
  	char text_buffer[MAX_BUFFER_SIZE]; // Buffer to hold the character to print
	int buffer_index = 0; // Current location in text_buffer
	int underline_ofst = 0; // Offset used for blinking underline
	bool firstCharReceived = false; // Flag to ignore residual reg value
	bool spaceFlag = false; // Flag to prevent multiple space chars
	bool underline_visible = false; // Flag to print and delete underline


	struct timeval blink_timer, last_update_time, current_time;
    gettimeofday(&last_update_time, NULL); // Initialize last update time
	gettimeofday(&blink_timer, NULL); // Initialize blink timer
	
   if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
	    printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
    
	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	
    // Set framebuffer addr to beginning of the SRAM
    PHYSMEM_32(0xff203024) = 0xc8000000;  	// Pixel BackBuffer register
    PHYSMEM_32(0xff203020) = 0xc8000000;	// Pixel Buffer register
    
    // Unmap registers region, map onchip ram region
    if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	//Remap VB for letter_decoded_out PIO
	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	
	// Get the address for the 'letter_decoded_out' PIO
    h2p_lw_letter_decoded_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LETTER_DECODED_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

	//VB mapping for VGA output
    virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_OCRAM_BASE );
	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	// Fixed char arrays holding information
   	char text_top_row[60] = "Altera DE1-SoC Morse Decoder by Erik Van Schijndel\0";
	char text_middle_row[60] = "Space char (   ) is 7 time units of silence\0";
	char text_bottom_row[40] = "Input short/long signals on KEY[0]\0";
	char text_decoder[40] = "Decoded phrase(s) below: \0";
	
	char morse_key_header[60] = "International morse code key below:\0";
	char morse_key_first_row[100] = "A = .-    G = --.   M = --    S = ...  Y = -.--   0 = -----  6 = -....\0";
	char morse_key_second_row[100] = "B = -...  H = ....  N = -.    T = -    Z = --..   1 = .----  7 = --...\0";
	char morse_key_third_row[100] = "C = -.-.  I = ..    O = ---   U = ..-             2 = ..---  8 = ---..\0";
	char morse_key_fourth_row[100] = "D = -..   J = .---  P = .--.  V = ...-            3 = ...--  9 = ----.\0";
	char morse_key_fifth_row[100] = "E = .     K = -.-   Q = --.-  W = .--             4 = ....-\0";
	char morse_key_sixth_row[100] = "F = ..-.  L = .-..  R = .-.   X = -..-            5 = .....\0";
	VGA_clear(virtual_base);
	
	//Page setup for decoding screen
	VGA_box(0, 0, 640,80,0x07E0, virtual_base);
	VGA_box(0,80, 20, 480, 0x07E0, virtual_base);
	VGA_box(620,80, 640, 480, 0x07E0, virtual_base);
	VGA_box(0,340, 640, 480, 0x07E0, virtual_base);
	
	//Instruction text
	VGA_text(15,2,text_top_row,virtual_base);
	VGA_text(20,5,text_middle_row,virtual_base);
	VGA_text(23,8,text_bottom_row,virtual_base);
	VGA_text(3,11,text_decoder,virtual_base);
	
	VGA_text(3,44,morse_key_header,virtual_base);
	VGA_text(3,46,morse_key_first_row,virtual_base);
	VGA_text(3,48,morse_key_second_row,virtual_base);
	VGA_text(3,50,morse_key_third_row,virtual_base);
	VGA_text(3,52,morse_key_fourth_row,virtual_base);
	VGA_text(3,54,morse_key_fifth_row,virtual_base);
	VGA_text(3,56,morse_key_sixth_row,virtual_base);
	
	memset(text_buffer, '\0', MAX_BUFFER_SIZE); //Allocate memory for text_buffer
	
	while(1) { // Run an infinite loop
        letter_decoded = *((int *)h2p_lw_letter_decoded_addr); // Read value
		gettimeofday(&current_time, NULL); // Get current time
        long time_diff = current_time.tv_sec - last_update_time.tv_sec; // Time difference for accessing space character
		
		// Blinking underline logic
        if (current_time.tv_sec - blink_timer.tv_sec > 0.5) { // Toggle every half second
            underline_visible = !underline_visible;
            blink_timer = current_time;
			underline_ofst = strlen(text_buffer);
            if (firstCharReceived) {
				VGA_draw_underline(25 + (underline_ofst * 8), 111, underline_visible, virtual_base); // Check location and update underline character
            }
        }

		// Check if 12 seconds have passed since the last character was added
		if (time_diff >= 10 && buffer_index < MAX_BUFFER_SIZE - 1 && spaceFlag == false) {
			printf("Space added to text_buffer!\n");
			VGA_draw_underline(25 + (strlen(text_buffer) * 8), 111, false, virtual_base);
			spaceFlag = true;
			text_buffer[buffer_index++] = ' '; // Add space to the buffer
			text_buffer[buffer_index] = '\0'; // Null terminate the buffer
			VGA_text(3, 13, text_buffer, virtual_base); // Update text on VGA
			gettimeofday(&last_update_time, NULL); // Update last update time
			VGA_draw_underline(25 + (strlen(text_buffer) * 8), 111, true, virtual_base);
		}
		//If the letter is not a duplicate & we have available reg space
        if (letter_decoded != prev_letter_decoded && buffer_index < MAX_BUFFER_SIZE - 1) { // Check if value changed
            char decoded_char;
            // Convert and store the decoded value
            if (letter_decoded >= 0 && letter_decoded <= 9) {
                // It's a number (0-9)
                decoded_char = '0' + letter_decoded; // Convert number to character
            } else if (letter_decoded >= 10 && letter_decoded <= 35) {
                // It's a letter (A-Z)
                decoded_char = 'A' + (letter_decoded - 10); // Convert to lowercase letter
            } else {
                // Invalid value
                decoded_char = '?'; 
            }
			// After first char has been received and flag already set
            if (firstCharReceived) {
				// Just before updating text_buffer, ensure underline is not visible
				VGA_draw_underline(25 + (strlen(text_buffer) * 8), 111, false, virtual_base);
                text_buffer[buffer_index++] = decoded_char;
				spaceFlag = false; //Prepare for next space
				gettimeofday(&last_update_time, NULL); // Update last update time
                text_buffer[buffer_index] = '\0'; // Null terminate text buffer
                VGA_text(3, 13, text_buffer, virtual_base); // Print updated text
				VGA_draw_underline(25 + (strlen(text_buffer) * 8), 111, true, virtual_base);
            } else {
                // If this is the first character, set the flag and initialize the buffer
                firstCharReceived = true; // Set flag for init value
                text_buffer[0] = decoded_char; // Reset first index of text buffer
                text_buffer[1] = '\0'; // Terminate new character
                VGA_text(3, 13, text_buffer, virtual_base); // Reprint text values
            }
			
            printf("Buffer: %s\n", text_buffer); //Print statement for debugging
            prev_letter_decoded = letter_decoded; // Store previous value
        }

        usleep(1000 * 10); // Short delay 
    }
	
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );

}
