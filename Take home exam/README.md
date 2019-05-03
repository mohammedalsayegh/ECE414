# ECE414
ECE414 Feedback 

In this report five technical methods of design for controllers for G(s) plant are investigated. The plant itself is given to us in a form of randomly generated outputs from the function ece414planttf.p  set by the date of our birthday, in this case the 28th of December. There will be 100 plant G(s) where the nominal plant must be located to design each controller. Robust system that can manage to control the 100 plant of G(s).

1. Root Locus
The first technique is root locus. By investigation the option of the PID type controller we decide which best fit controller is to be designed. Because of the number zero’s in the plant only types PID and PIDF are feasible to be designed. A PI controller was chosen as candidate based on what was given as nominal plants.

2. pidtune and pidtuner
In this part, only the candidates PI and PIDF were considered for design on the bases of the possibility of PID that can achieved goal specification. A pidTuner use to generate the PID type plant. While pidTuner is used to open GUI interface and control the parameters of the PI and PIDF controller.

3. Unity Feedback Linear Algebra design
In this part of the report, a stepitae and stepshape were used to generate a D(s) plant to conrol the nominal G(s) plant. With the help of the lamdesign function, the result of the generated controller plan was generated and check if it met the unity feedback deign limitation.

4. Two Parameter Linear Algebraic Controller Design
For two parameters LAM, lamdesign function was used. A plant generated using steplqr, the real roots of  the plant used as vector to pass  lamdesign to generate H(s) and F(s) plants.
