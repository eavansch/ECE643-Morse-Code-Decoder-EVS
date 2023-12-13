# ECE643-Morse-Code-Decoder-EVS
Final project for ECE 643 - Cmp Eng Dsgn Lab at Kansas State University implemented on the Altera DE1-SoC board.

Project is an implementation of the provided "DE1-SoC_Computer_KSU.zip" project files provided by K-State. Utilizing KEY[0] on our board, users can either input a long/short pulse which is then stored in bit array and then decoded. 
This decoded int is passed to the seven-segment displays onboard the DE1-SoC, and is displayed as users decoded more chars/num.

This decoded int is then passed through the H2F LW AXI bridge, to then communicate with our compiled HPS C program. This C program converts our decoded buffer into an ASCII character/number, before outputting to a VGA screen.

Demo video is here; https://youtu.be/MbgpOAnW2Sw
