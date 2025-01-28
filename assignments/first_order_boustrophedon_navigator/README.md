# First-Order Boustrophedon Navigator

### OUTPUT PIC:

![TURTLE PATTERN](https://github.com/user-attachments/assets/6107e4fe-3038-4447-bc8b-fcb059885d90)

### EXPLANATION:

#### **Kp_linear (Proportional Gain for Linear Velocity)**
- **Reduced from:** 10.0 to 2.8.
- **Impact:** The turtle moved smoother toward waypoints with reduced overshooting and oscillations. The slower approach made sure the travel became more stable but increased the time it took to reach each waypoint a little bit.

#### **Kd_linear (Derivative Gain for Linear Velocity)**
- **Reduced from:** 0.1 to 0.05.
- **Impact:** It avoided excessive damping; therefore, it succeeded in maintaining a forward speed in motion without many oscillatory corrections.

#### **Kp_angular (Proportional Gain for Angular Velocity)**
- **Increased from:** 5.0 to 12.0.
- **Effect:** The turtle became more responsive to angular deviations, which helped to reach the target orientation faster. Thus, cornering performance improved but with a bit higher risk of oscillations.

#### **Kd_angular (Derivative Gain for Angular Velocity)**
- **Reduced from:** 0.2 to 0.008.
- **Effect:** The angular damping of the turtle was reduced, enabling faster corrections of the angle. This helped to increase responsiveness, but it was a delicate setting that required much tuning to prevent instability.

#### **spacing (Distance Between Rows in the Lawn Mower Pattern)**
- **Changed:** Increased from 1.0 to 1.4.
- **Effect:** The turtle covered the area with fewer rows, completing the pattern faster. This made the coverage a little sparser, which was acceptable for this simulation.

### **Received Error Margins:**
- **Average cross-track error:** 0.039
- **Maximum cross-track error:** 0.173

---

### **NOTE:**
- Extra Credit questions were not done by me.
- I tried to solve them but faced issues in creating the `CMakeLists.txt` file. 
- I will work on the extra credit assignment and, as soon as I am done with it, I will update the code and `README.md` file in the GitHub repository.

---

### **GDrive Link:**
[Assignment Files](https://drive.google.com/drive/folders/1AeyUbpCp7VYYvtE1hp4Lb87LZvL0DXom?usp=sharing)
