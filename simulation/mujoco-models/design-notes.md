## MuJoCo Model Design Decisions

To guide the design of my robotic hand, I built a simplified MuJoCo model focused on essential grasping motions using a two-finger and one-thumb configuration.

### 1. **Finger and Thumb Positioning**

I positioned the fingers based on my own relaxed hand posture—fingers spread in a single plane and slightly angled outward. The thumb was placed similarly, allowing it to oppose the fingers both:

* **Forward**, toward the fingertips (for cylindrical or tripod pinch grasps)
* **Laterally**, toward the side of the first finger (for diagonal volar grips)

To enable this, I added a degree of rotation at the **base of the thumb**, allowing it to pivot about the center of the palm. However, I realized a second rotational degree of freedom—where the **proximal phalange meets the thumb’s metacarpal**—may be necessary for more versatile opposition.

### 2. **Phalange Count and Control Simplicity**

I limited the design to **two phalanges per digit** (fingers and thumb) to simplify both physical prototyping and control logic, while still enabling meaningful grasp types.

### 3. **Initial Tendon-Driven Approach**

Initially, I aimed for a **tendon-driven control system**:

* I defined *sites* along the top and bottom of each phalange
* **Strings (tendons)** were to simulate flexion via actuators
* **Springs** were used for passive extension

Despite tuning stiffness values and tendon lengths, the fingers didn’t move in simulation—even when manually adjusting joint and actuator forces. Based on advice from Huy, I decided to switch to **direct joint actuators**. When designing the physical hand, I will then ensure I am able to match the position of the hand in simulation.

