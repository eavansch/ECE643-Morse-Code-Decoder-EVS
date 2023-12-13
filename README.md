# ECE643-Morse-Code-Decoder-EVS
Final project for ECE 643 - Cmp Eng Dsgn Lab at Kansas State University implemented on the Altera DE1-SoC board.

Project is an implementation of the provided "DE1-SoC_Computer_KSU.zip" project files provided by K-State. Utilizing KEY[0] on our board, users can either input a long/short pulse which is then stored in bit array and then decoded. 
This decoded int is passed to the seven-segment displays onboard the DE1-SoC, and is displayed as users decoded more chars/num.

This decoded int is then passed through the H2F LW AXI bridge, to then communicate with our compiled HPS C program. This C program converts our decoded buffer into an ASCII character/number, before outputting to a VGA screen.

Demo video is here; https://youtu.be/MbgpOAnW2Sw

System Diagram of project:
<img width="888" alt="Drawing2 (1)" src="https://github.com/eavansch/ECE643-Morse-Code-Decoder-EVS/assets/89333755/d9c64d16-3f2a-445d-82c4-b8f0bb88ee10">

Sample output from validation testing:
![IMG_1384](https://github.com/eavansch/ECE643-Morse-Code-Decoder-EVS/assets/89333755/b2677d37-b8ea-4985-a1a1-2235733bd652)
