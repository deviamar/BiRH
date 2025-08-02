## Using AHAP to Guide Robotic Hand Design

While exploring ways to evaluate and refine robotic hand designs, I discovered the *Anthropomorphic Hand Assessment Protocol* (AHAP) (Llop-Harillo et al., 2019), which proposes a standardized framework for evaluating robotic hands using common grasp types from activities of daily living (ADLs). This aligned perfectly with my goal of systematically improving prototypes through understanding essential grasp configurations.

**Adapted Testing Methodology:**
To test my design ideas using these principles, I developed a low-cost validation approach by using constrained motions of my own hand to simulate different robotic configurations while attempting AHAP-defined grasps with simple objects (wire, paper, tape).

**Configuration Testing:**
- **Prototype 1**: Three fingers, three phalanges, all hinge joints
- **Config 1**: Two fingers + thumb, all hinge joints
- **Config 2**: Two fingers + thumb with thumb abduction/adduction

### Configuration Testing Results

| Grasp Type | Prototype 1 | Config 1 | Config 2 | Notes |
|------------|------------ | ---------|----------|-------|
| Spherical grip | ✓ Stable | ⚠️ Less stable | ⚠️ Less stable |
| Cylindrical grip | ⚠️ Poor contact | ✓ More Stable | ⚠️ Less stable |
| Diagonal volar grip | ✗ Not possible | ✗ Not possible | ✓ Stable |
| Lateral pinch | ⚠️ Not stable | ✗ Not possible | ✓ Stable |
| Extension grip | ✓ Possible | ✓ Stable | ✓ Stable |
| Platform | ✓ Possible | ✓ Possible | ✓ Possible |
| Tripod pinch | ✓ Stable | ✓ Stable | ✓ Stable |
| Pulp pinch | ✓ Stable | ✓ Stable | ✓ Stable |
| Hook | ✓ Possible | ✓ More Stable | ✓ Stable |

See images and detailed notes here: [Test Results](Test.pdf)

**Key Design Insights:**
- **Surface area contact** and **opposing finger orientation** critically affect stability
- **Missing abduction/adduction** eliminates entire grasp categories (lateral pinch, diagonal volar grip)

**Impact on Next Prototype:**
Based on these results, I decided to develop a second prototype where I have two fingers opposing a thumb. Each finger has three phalanges and the thumb is capable of opposing in two directions. Toward the fingertips for cylindrical and tripod pinch grasps, and toward the side of the first finger to enable a diagonal volar grip.

## References

Llop-Harillo, I., Pérez-González, A., Starke, J., & Asfour, T. (2019). The Anthropomorphic Hand Assessment Protocol (AHAP). *Robotics and Autonomous Systems*, 121, 103259. https://doi.org/10.1016/j.robot.2019.103259
